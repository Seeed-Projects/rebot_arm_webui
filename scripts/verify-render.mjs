import { spawn } from "node:child_process";
import { setTimeout as delay } from "node:timers/promises";

const url = process.argv[2] ?? "http://localhost:5173/";
const port = 9222;
const chrome = spawn("google-chrome", [
  "--headless=new",
  "--no-sandbox",
  "--use-angle=swiftshader",
  "--use-gl=angle",
  "--enable-unsafe-swiftshader",
  `--remote-debugging-port=${port}`,
  "--window-size=1280,900",
  url,
], { stdio: "ignore" });

async function getJson(endpoint) {
  const response = await fetch(endpoint);
  if (!response.ok) {
    throw new Error(`${endpoint} returned ${response.status}`);
  }
  return response.json();
}

async function waitForDebugger() {
  for (let i = 0; i < 50; i += 1) {
    try {
      return await getJson(`http://127.0.0.1:${port}/json`);
    } catch {
      await delay(100);
    }
  }
  throw new Error("Chrome DevTools endpoint did not start");
}

async function send(ws, method, params = {}) {
  const id = send.nextId++;
  ws.send(JSON.stringify({ id, method, params }));
  return new Promise((resolve, reject) => {
    const listener = (event) => {
      const message = JSON.parse(event.data);
      if (message.id !== id) return;
      ws.removeEventListener("message", listener);
      if (message.error) reject(new Error(message.error.message));
      else resolve(message.result);
    };
    ws.addEventListener("message", listener);
  });
}
send.nextId = 1;

try {
  const pages = await waitForDebugger();
  const page = pages.find((entry) => entry.type === "page") ?? pages[0];
  const ws = new WebSocket(page.webSocketDebuggerUrl);
  await new Promise((resolve) => ws.addEventListener("open", resolve, { once: true }));

  await send(ws, "Runtime.enable");
  await send(ws, "Page.enable");
  const browserMessages = [];
  ws.addEventListener("message", (event) => {
    const message = JSON.parse(event.data);
    if (message.method === "Runtime.consoleAPICalled") {
      browserMessages.push(message.params.args.map((arg) => arg.value ?? arg.description).join(" "));
    }
    if (message.method === "Runtime.exceptionThrown") {
      browserMessages.push(message.params.exceptionDetails.text);
    }
  });
  await send(ws, "Page.navigate", { url });
  await delay(5000);

  const result = await send(ws, "Runtime.evaluate", {
    returnByValue: true,
    expression: `
      (() => {
        const canvas = document.querySelector("#armViewport canvas");
        const overlay = document.querySelector("#viewportOverlay");
        if (!canvas) {
          return {
            ok: false,
            reason: "canvas missing",
            title: document.title,
            body: document.body?.innerText?.slice(0, 300) ?? "",
            scripts: [...document.scripts].map((script) => script.src || script.textContent.slice(0, 40)),
          };
        }
        const context = canvas.getContext("webgl2") || canvas.getContext("webgl");
        if (!context) return { ok: false, reason: "webgl readback unavailable" };
        const width = canvas.width;
        const height = canvas.height;
        const data = new Uint8Array(width * height * 4);
        context.readPixels(0, 0, width, height, context.RGBA, context.UNSIGNED_BYTE, data);
        let nonBlank = 0;
        let darkPixels = 0;
        for (let i = 0; i < data.length; i += 4) {
          if (data[i] !== 232 || data[i + 1] !== 237 || data[i + 2] !== 241) {
            nonBlank += 1;
          }
          if (data[i] < 170 && data[i + 1] < 180 && data[i + 2] < 190) {
            darkPixels += 1;
          }
        }
        const debug = window.__rebotArmDebug;
        return {
          ok: Boolean(debug?.robotLoaded) && darkPixels > width * height * 0.01,
          width,
          height,
          nonBlank,
          darkPixels,
          overlay: overlay?.textContent ?? ""
          ,
          overlayHidden: Boolean(overlay?.hidden),
          robotLoaded: Boolean(debug?.robotLoaded),
          joints: debug?.jointNames ?? [],
          links: debug?.linkNames ?? [],
          sceneBounds: debug?.sceneBounds ?? null,
          tcpText: document.querySelector("#tcpText")?.textContent ?? "",
          rpyText: document.querySelector("#rpyText")?.textContent ?? ""
        };
      })()
    `,
  });

  const value = result.result.value;
  console.log(JSON.stringify({ ...value, browserMessages }, null, 2));
  if (!value.ok) {
    process.exitCode = 1;
  }
  ws.close();
} finally {
  chrome.kill("SIGTERM");
}
