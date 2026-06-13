import { spawn } from "node:child_process";
import { setTimeout as delay } from "node:timers/promises";

const url = process.argv[2] ?? "http://localhost:5173/";
const port = 9225;
const chrome = spawn("google-chrome", [
  "--headless=new",
  "--no-sandbox",
  "--use-angle=swiftshader",
  "--use-gl=angle",
  "--enable-unsafe-swiftshader",
  `--remote-debugging-port=${port}`,
  "--window-size=1440,1000",
  url,
], { stdio: "ignore" });

async function getJson(endpoint) {
  const response = await fetch(endpoint);
  if (!response.ok) throw new Error(`${endpoint} returned ${response.status}`);
  return response.json();
}

async function waitForPage() {
  for (let i = 0; i < 50; i += 1) {
    try {
      const pages = await getJson(`http://127.0.0.1:${port}/json`);
      return pages.find((page) => page.type === "page") ?? pages[0];
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
  const page = await waitForPage();
  const ws = new WebSocket(page.webSocketDebuggerUrl);
  await new Promise((resolve) => ws.addEventListener("open", resolve, { once: true }));
  await send(ws, "Runtime.enable");
  await send(ws, "Page.enable");
  await send(ws, "Page.navigate", { url });
  await delay(5000);
  const result = await send(ws, "Runtime.evaluate", {
    returnByValue: true,
    expression: `
      (() => {
        const values = [...document.querySelectorAll("#jointList .joint-value")].map(el => el.textContent);
        return { values, ok: values.every(value => /^-?0\\.0°$/.test(value)) };
      })()
    `,
  });
  console.log(JSON.stringify(result.result.value, null, 2));
  if (!result.result.value.ok) process.exitCode = 1;
  ws.close();
} finally {
  chrome.kill("SIGTERM");
}
