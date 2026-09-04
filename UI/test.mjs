import assert from "node:assert/strict";
import { mkdtemp, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { spawn } from "node:child_process";

const profile = await mkdtemp(join(tmpdir(), "fieldnote-ui-"));
const browser = spawn(process.env.CHROME || "google-chrome", [
  "--headless", "--no-sandbox", "--disable-gpu", "--disable-extensions",
  `--user-data-dir=${profile}`, "--remote-debugging-port=0",
  new URL("index.html#datasets", import.meta.url).href,
], { stdio: ["ignore", "ignore", "pipe"] });

try {
  const browserUrl = await new Promise((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error("Chrome startup timed out")), 10_000);
    browser.stderr.on("data", (chunk) => {
      const match = chunk.toString().match(/DevTools listening on (ws:\/\/\S+)/);
      if (match) { clearTimeout(timeout); resolve(match[1]); }
    });
    browser.once("error", reject);
  });
  const { port } = new URL(browserUrl);
  const pages = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
  const page = pages.find((item) => item.type === "page" && item.url.includes("/UI/index.html"));
  assert(page, "UI page was not opened");

  const socket = new WebSocket(page.webSocketDebuggerUrl);
  await new Promise((resolve) => socket.addEventListener("open", resolve, { once: true }));
  let sequence = 0;
  const evaluate = (expression) => new Promise((resolve, reject) => {
    const id = ++sequence;
    const receive = ({ data }) => {
      const message = JSON.parse(data);
      if (message.id !== id) return;
      socket.removeEventListener("message", receive);
      if (message.result.exceptionDetails) reject(new Error(message.result.exceptionDetails.exception?.description || message.result.exceptionDetails.text));
      else resolve(message.result.result.value);
    };
    socket.addEventListener("message", receive);
    socket.send(JSON.stringify({ id, method: "Runtime.evaluate", params: { expression, awaitPromise: true, returnByValue: true } }));
  });

  await evaluate('new Promise(r=>{const wait=setInterval(()=>{if(document.readyState==="complete"&&typeof askDelete==="function"){clearInterval(wait);r(true)}},20)})');
  assert.deepEqual(
    await evaluate('({sections:document.querySelectorAll("#view-workbench .usage-guide section").length,pico:document.querySelector("#view-workbench .usage-guide").textContent.includes("Network=WORKING"),enter:document.querySelector("#view-workbench .usage-guide").textContent.includes("Enter")})'),
    { sections: 2, pico: true, enter: true },
  );
  await evaluate('renderEpisodes([{id:"episode_131937_81295016",task:"pick_and_place",operator:"zxcx",robot_model:"M6S-Lite-CCS-680-B",status:"degraded",duration_seconds:115,size_bytes:5583457485,created_at:"2026-09-03T13:19:00+08:00",modalities:["关节","PICO","触觉","视觉"]}])');

  assert.deepEqual(
    await evaluate('document.querySelector(".episode-select").click();({selected:exportCount.textContent,enabled:!exportMcap.disabled,detail:episodeDialog.open})'),
    { selected: "1", enabled: true, detail: false },
  );
  assert.equal(await evaluate('document.querySelector("#exportMcap").click();document.querySelector("#toast span").textContent'), "导出 MCAP 需要启动 UI 后端");
  assert.deepEqual(
    await evaluate('document.querySelector("#datasetRows [data-episode]").click();openDirectoryFromDetail.click();new Promise(r=>setTimeout(()=>r({id:detailId.textContent,message:document.querySelector("#toast span").textContent}),20))'),
    { id: "episode_131937_81295016", message: "打开目录需要启动 UI 后端" },
  );
  await evaluate('episodeDialog.close()');
  assert.deepEqual(
    await evaluate('document.querySelector(".delete-button").click();({open:deleteDialog.open,target:deleteTarget.textContent})'),
    { open: true, target: "episode_131937_81295016" },
  );
  assert.deepEqual(
    await evaluate('removeEpisode(deleteTarget.textContent);({rows:datasetRows.rows.length,count:datasetCount.textContent})'),
    { rows: 1, count: "0" },
  );
  assert.deepEqual(
    await evaluate('openView("workbench");visionResolution.value="1600x1296";visionResolution.dispatchEvent(new Event("change"));document.querySelector(".camera-format").textContent'),
    "1600 × 1296 · 目标 60 Hz",
  );
  assert.equal(await evaluate('document.querySelector(".camera-scene").getBoundingClientRect().height >= 340'), true);
  assert.deepEqual(await evaluate('({images:document.querySelectorAll(".camera-preview").length,active:document.querySelectorAll(".preview-ready").length})'), { images: 2, active: 0 });
  assert.deepEqual(await evaluate('({marvin:marvinStreamHealth.textContent,das:dasStreamHealth.textContent,vision:visionHealth.textContent})'), { marvin: "离线", das: "离线", vision: "离线" });
  assert.deepEqual(
    await evaluate('({offline:picoConnection.classList.contains("disconnected"),disabled:startButton.disabled})'),
    { offline: true, disabled: true },
  );
  assert.equal(await evaluate('monitorErrors.classList.contains("clear")'), false);
  assert.equal(await evaluate('picoConnection.click();new Promise(r=>setTimeout(()=>r(picoConnection.classList.contains("disconnected")),20))'), true);
  await evaluate('setPicoStatus("connected", {service_ready:true,ports_listening:[60061,63901],clients:["192.168.1.42"]})');
  assert.equal(await evaluate('monitorErrors.classList.contains("clear")'), true);
  assert.equal(await evaluate('collectionError="遥操进程异常退出：测试故障";renderMonitorErrors();monitorErrorList.textContent.includes("测试故障")'), true);
  await evaluate('collectionError="";renderMonitorErrors()');
  assert.deepEqual(
    await evaluate('document.querySelector("[name=task]").dispatchEvent(new KeyboardEvent("keydown",{key:"Enter",bubbles:true,cancelable:true}));({confirmation:!!document.querySelector("#safetyDialog"),recording:startButton.classList.contains("recording"),resetEnabled:!document.querySelector("#resetRobot").disabled})'),
    { confirmation: false, recording: true, resetEnabled: false },
  );
  assert.equal(await evaluate('requestRobotReset();document.querySelector("#toast span").textContent'), "机器人复位需要启动 UI 后端");
  await evaluate('startButton.click()');
  socket.close();
  console.log("UI interaction check passed");
} finally {
  if (browser.exitCode === null && browser.signalCode === null) {
    await new Promise((resolve) => {
      browser.once("exit", resolve);
      browser.kill("SIGTERM");
    });
  }
  await rm(profile, { recursive: true, force: true, maxRetries: 5, retryDelay: 50 });
}
