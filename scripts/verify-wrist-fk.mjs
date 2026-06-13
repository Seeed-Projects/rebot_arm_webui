import { spawn } from "node:child_process";
import { setTimeout as delay } from "node:timers/promises";

const url = process.argv[2] ?? "http://localhost:5173/";
const port = 9223;
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
      (async () => {
        const rows = [...document.querySelectorAll("#jointList .joint-row")];
        const readPose = () => Object.fromEntries(
          [...document.querySelectorAll("#cartesianForm input")]
            .map(input => [input.dataset.axis, Number(input.value)])
        );
        const results = [];
        for (const index of [3, 4, 5]) {
          const input = rows[index]?.querySelector("input");
          const before = readPose();
          input.value = String(Number(input.value) + 35);
          input.dispatchEvent(new InputEvent("input", { bubbles: true }));
          await new Promise(resolve => setTimeout(resolve, 700));
          const after = readPose();
          const delta = Math.hypot(after.x - before.x, after.y - before.y, after.z - before.z);
          results.push({ joint: \`J\${index + 1}\`, before, after, delta, changed: delta > 0.0005 });
        }
        return { results, changed: results.some(item => item.changed) };
      })()
    `,
    awaitPromise: true,
  });

  console.log(JSON.stringify(result.result.value, null, 2));
  if (!result.result.value.changed) process.exitCode = 1;
  ws.close();
} finally {
  chrome.kill("SIGTERM");
}
