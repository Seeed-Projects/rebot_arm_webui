import { spawn } from "node:child_process";
import { setTimeout as delay } from "node:timers/promises";

const url = process.argv[2] ?? "http://localhost:5173/";
const port = 9224;
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
    awaitPromise: true,
    expression: `
      (async () => {
        const jointValues = () => [...document.querySelectorAll("#jointList .joint-value")].map(el => el.textContent);
        const before = jointValues();
        const values = { x: 0.20, y: 0.02, z: 0.30, roll: 0, pitch: 0, yaw: 0 };
        for (const [axis, value] of Object.entries(values)) {
          const input = document.querySelector(\`#cartesianForm input[data-axis="\${axis}"]\`);
          input.value = String(value);
          input.dispatchEvent(new InputEvent("input", { bubbles: true }));
        }
        document.querySelector("#solveIkButton").click();
        await new Promise(resolve => setTimeout(resolve, 3000));
        const after = jointValues();
        return {
          before,
          after,
          changed: JSON.stringify(before) !== JSON.stringify(after),
          solverStatus: document.querySelector("#solverStatus")?.textContent ?? "",
          tcp: document.querySelector("#tcpText")?.textContent ?? ""
        };
      })()
    `,
  });

  console.log(JSON.stringify(result.result.value, null, 2));
  if (!result.result.value.changed) process.exitCode = 1;
  ws.close();
} finally {
  chrome.kill("SIGTERM");
}
