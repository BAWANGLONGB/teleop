import assert from "node:assert/strict";
import { mkdtemp, readFile, rm } from "node:fs/promises";
import { createServer } from "node:http";
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
  const page = pages.find((item) => item.type === "page" && item.url.includes("/ui/index.html"));
  assert(page, "UI page was not opened");

  const socket = new WebSocket(page.webSocketDebuggerUrl);
  await new Promise((resolve) => socket.addEventListener("open", resolve, { once: true }));
  let sequence = 0;
  const send = (method, params = {}) => new Promise((resolve, reject) => {
    const id = ++sequence;
    const receive = ({ data }) => {
      const message = JSON.parse(data);
      if (message.id !== id) return;
      socket.removeEventListener("message", receive);
      if (message.error) reject(new Error(message.error.message));
      else resolve(message.result);
    };
    socket.addEventListener("message", receive);
    socket.send(JSON.stringify({ id, method, params }));
  });
  const evaluate = async (expression) => {
    const result = await send("Runtime.evaluate", { expression, awaitPromise: true, returnByValue: true });
    if (result.exceptionDetails) throw new Error(result.exceptionDetails.exception?.description || result.exceptionDetails.text);
    return result.result.value;
  };

  await evaluate('new Promise(r=>{const wait=setInterval(()=>{if(document.readyState==="complete"&&typeof askDelete==="function"){clearInterval(wait);r(true)}},20)})');
  assert.deepEqual(
    await evaluate('({sections:document.querySelectorAll("#view-workbench .usage-guide section").length,pico:document.querySelector("#view-workbench .usage-guide").textContent.includes("Network=WORKING"),devices:document.querySelector("#view-workbench .usage-guide").textContent.includes("启动设备")})'),
    { sections: 2, pico: true, devices: true },
  );
  await evaluate('window.testEpisodes=[{id:"episode_131937_81295016",session:"session_2026-09-03",task:"pick_and_place",operator:"zxcx",robot_model:"M6S-Lite-CCS-680-B",status:"degraded",duration_seconds:115,size_bytes:5583457485,created_at:"2026-09-03T13:19:00+08:00",modalities:["关节","PICO","触觉","视觉"]},{id:"episode_131938_cafebabe",session:"session_2026-09-04",task:"stack_blocks",operator:"zxcx",robot_model:"M6S-Lite-CCS-680-B",status:"completed",duration_seconds:60,size_bytes:1024,created_at:"2026-09-04T13:19:00+08:00",modalities:["关节"]}];renderEpisodes(testEpisodes)');
  assert.deepEqual(
    await evaluate('sessionFilter.value="session_2026-09-03";sessionFilter.dispatchEvent(new Event("change"));({options:sessionFilter.options.length,visible:[...datasetRows.rows].filter(row=>!row.hidden).map(row=>row.dataset.session)})'),
    { options: 3, visible: ["session_2026-09-03"] },
  );
  await evaluate('collectionSession.add(new Option("当前 Session",testEpisodes[0].session));collectionSession.value=testEpisodes[0].session;collectionSession.dispatchEvent(new Event("change"))');
  assert.equal(await evaluate('datasetCount.textContent'), "1");
  assert.equal(await evaluate('renderEpisodes([...testEpisodes,testEpisodes[0]]);datasetCount.textContent'), "2");
  assert.equal(await evaluate('lastCollection={active:true,session:testEpisodes[1].session};updateDatasetCount();datasetCount.textContent'), "1");
  assert.equal(await evaluate('lastCollection={};collectionSession.value="";collectionSession.dispatchEvent(new Event("change"));const d=new Date();const today=`session_${d.getFullYear()}-${String(d.getMonth()+1).padStart(2,"0")}-${String(d.getDate()).padStart(2,"0")}`;renderEpisodes([{...testEpisodes[0],session:today},{...testEpisodes[1],session:"session_old"}]);datasetCount.textContent'), "1");
  await evaluate('renderEpisodes([testEpisodes[0]])');
  assert.equal(await evaluate('document.querySelector("#view-datasets thead").textContent.includes("状态")'), false);
  assert.deepEqual(await evaluate('({workbench:document.querySelectorAll("#view-workbench [data-review-result]").length,datasets:document.querySelectorAll("#view-datasets [data-review-result]").length})'), {workbench:3,datasets:0});
  assert.equal(await evaluate('collectionSession.add(new Option("测试 Session","session_test"));collectionSession.value="session_test";collectionPayload().session'), "session_test");
  assert.equal(await evaluate('collectionSession.dispatchEvent(new Event("change"));datasetCount.textContent'), "0");
  await evaluate('collectionSession.value="";lastCollection={episode_id:testEpisodes[0].id,session:testEpisodes[0].session,active:true,episode_exists:true,review:{result:"unmarked"}};renderReviewEpisodes()');
  assert.equal(await evaluate('[...document.querySelectorAll("[data-review-result]")].every(b=>b.disabled)'), true);
  await evaluate('lastCollection.active=false;lastCollection.review={result:"success"};renderReviewControls()');
  assert.equal(await evaluate('reviewResult.textContent'), "人工结果：成功");
  assert.equal(await evaluate('document.querySelector("[data-review-result=success]").getAttribute("aria-pressed")'), "true");
  await evaluate('lastCollection.review={result:"unmarked"};renderReviewControls()');
  assert.equal(await evaluate('reviewResult.textContent'), "人工结果：成功（默认）");
  assert.equal(await evaluate('datasetRows.textContent.includes("成功（默认）")'), true);
  assert.equal(await evaluate('document.querySelector("#collectionForm").textContent.includes("X 开始/结束录制")'), false);
  assert.equal(await evaluate('document.querySelector("#controllerStatus")'), null);
  await evaluate('lastCollection.episode_exists=false;renderReviewEpisodes()');
  assert.equal(await evaluate('reviewCandidates().get(`${testEpisodes[0].session}/${testEpisodes[0].id}`).can_review'), undefined);
  await evaluate('document.querySelector("[data-review-result=failure]").click()');
  assert.equal(await evaluate('document.querySelector("#toast span").textContent'), "结果标注需要启动 UI 后端");
  await evaluate('lastCollection={};renderReviewEpisodes()');

  assert.deepEqual(
    await evaluate('document.querySelector(".episode-select").click();({selected:exportCount.textContent,enabled:!exportMcap.disabled,detail:episodeDialog.open})'),
    { selected: "1", enabled: true, detail: false },
  );
  assert.deepEqual(
    await evaluate('(async()=>{const events=[];const directory={getFileHandle:async name=>({createWritable:async()=>new WritableStream({close(){events.push(`done:${name}`)}})})};await exportEpisodeMcaps(["episode_120000_deadbeef","episode_120001_cafebabe"],directory,async (url,options)=>{if(options?.method==="POST"){events.push(`pack:${JSON.parse(options.body).episode}`);return new Response("{}");}const id=url.searchParams.get("episode");events.push(id);return new Response("av1",{headers:{"Content-Disposition":`attachment; filename="${id}.av1.mcap"`}})});return events})()'),
    ["pack:episode_120000_deadbeef", "episode_120000_deadbeef", "done:episode_120000_deadbeef.av1.mcap", "pack:episode_120001_cafebabe", "episode_120001_cafebabe", "done:episode_120001_cafebabe.av1.mcap"],
  );
  assert.equal(await evaluate('(async()=>{try{await exportEpisodeMcaps(["episode_120000_deadbeef"],{getFileHandle(){throw new Error("must not write")}},async()=>new Response("mjpeg",{headers:{"Content-Disposition":"attachment; filename=\\"episode_120000_deadbeef.mjpeg.mcap\\""}}));return false}catch(e){return e.message.includes("后端未返回 AV1 MCAP")}})()'), true);
  assert.deepEqual(await evaluate('[...exportFormat.options].map(o=>o.value)'), ["av1", "h264", "mjpeg"]);
  assert.deepEqual(await evaluate('(async()=>{const files=[];await exportEpisodeMcaps(["episode_120000_deadbeef"],{getFileHandle:async name=>{files.push(name);return {createWritable:async()=>new WritableStream()}}},async (url,options)=>{if(options?.method==="POST"){files.push(JSON.parse(options.body).format);return new Response("{}")}return new Response("jpeg",{headers:{"Content-Disposition":`attachment; filename="episode_120000_deadbeef.mjpeg.mcap"`}})},"mjpeg");return files})()'), ["mjpeg", "episode_120000_deadbeef.mjpeg.mcap"]);
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
    await evaluate('({offline:picoConnection.classList.contains("disconnected"),devicesDisabled:deviceButton.disabled,recordDisabled:recordButton.disabled})'),
    { offline: true, devicesDisabled: true, recordDisabled: true },
  );
  assert.equal(await evaluate('monitorErrors.classList.contains("clear")'), false);
  assert.equal(await evaluate('picoConnection.click();new Promise(r=>setTimeout(()=>r(picoConnection.classList.contains("disconnected")),20))'), true);
  await evaluate('setPicoStatus("connected", {service_ready:true,ports_listening:[60061,63901],clients:["192.168.1.42"]})');
  assert.equal(await evaluate('monitorErrors.classList.contains("clear")'), true);
  assert.equal(await evaluate('collectionError="遥操进程异常退出：测试故障";renderMonitorErrors();monitorErrorList.textContent.includes("测试故障")'), true);
  await evaluate('collectionError="";renderMonitorErrors()');
  assert.deepEqual(
    await evaluate('deviceButton.click();({devices:deviceButton.classList.contains("devices-active"),recordEnabled:!recordButton.disabled})'),
    { devices: true, recordEnabled: true },
  );
  assert.deepEqual(
    await evaluate('document.querySelector("[name=task]").dispatchEvent(new KeyboardEvent("keydown",{key:"Enter",bubbles:true,cancelable:true}));({confirmation:!!document.querySelector("#safetyDialog"),recording:recordButton.classList.contains("recording"),deviceDisabled:deviceButton.disabled,resetEnabled:!document.querySelector("#resetRobot").disabled})'),
    { confirmation: false, recording: true, deviceDisabled: true, resetEnabled: false },
  );
  assert.equal(await evaluate('requestRobotReset();document.querySelector("#toast span").textContent'), "机器人复位需要启动 UI 后端");
  await evaluate('recordButton.click();deviceButton.click()');
  await evaluate('reviewEpisode.add(new Option("长 Session 名称测试".repeat(8) + " / episode_120000_deadbeef", "layout-test"));reviewEpisode.value="layout-test"');
  for (const width of [1440, 390]) {
    await send("Emulation.setDeviceMetricsOverride", { width, height: 900, deviceScaleFactor: 1, mobile: false });
    assert.deepEqual(await evaluate(`(() => {
      const review = document.querySelector(".review-box");
      const bounds = review.getBoundingClientRect();
      const select = reviewEpisode.getBoundingClientRect();
      return {
        inMonitor: review.parentElement.classList.contains("monitor-panel"),
        belowMonitor: review.previousElementSibling === monitorErrors && bounds.top >= monitorErrors.getBoundingClientRect().bottom,
        outsideConfig: !review.closest(".launch-panel"),
        fits: bounds.width > 0 && bounds.right <= innerWidth && select.left >= bounds.left && select.right <= bounds.right,
      };
    })()`), { inMonitor: true, belowMonitor: true, outsideConfig: true, fits: true }, `review layout at ${width}px`);
  }
  await send("Emulation.setDeviceMetricsOverride", { width: 1440, height: 900, deviceScaleFactor: 1, mobile: false });
  await evaluate('new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)))');
  assert.deepEqual(await evaluate('menuButton.click();({expanded:menuButton.getAttribute("aria-expanded"),hidden:getComputedStyle(sidebar).display==="none",inert:sidebar.inert,margin:getComputedStyle(document.querySelector(".app")).marginLeft,label:menuButton.getAttribute("aria-label")})'),
    { expanded: "false", hidden: true, inert: true, margin: "0px", label: "展开导航" });
  await evaluate('openView("datasets")');
  assert.equal(await evaluate('getComputedStyle(sidebar).display'), "none");
  assert.deepEqual(await evaluate('menuButton.click();({expanded:menuButton.getAttribute("aria-expanded"),visible:getComputedStyle(sidebar).display!=="none",inert:sidebar.inert,margin:getComputedStyle(document.querySelector(".app")).marginLeft})'),
    { expanded: "true", visible: true, inert: false, margin: "232px" });
  await send("Emulation.setDeviceMetricsOverride", { width: 390, height: 900, deviceScaleFactor: 1, mobile: false });
  await evaluate('new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)))');
  assert.equal(await evaluate('menuButton.getAttribute("aria-expanded")'), "false");
  assert.deepEqual(await evaluate('menuButton.click();({open:sidebar.classList.contains("open"),inert:sidebar.inert,expanded:menuButton.getAttribute("aria-expanded")})'),
    { open: true, inert: false, expanded: "true" });
  await evaluate('document.querySelector(".nav-item[data-nav=workbench]").click()');
  assert.deepEqual(await evaluate('({open:sidebar.classList.contains("open"),inert:sidebar.inert,expanded:menuButton.getAttribute("aria-expanded")})'),
    { open: false, inert: true, expanded: "false" });
  // Serve the real UI over HTTP, with hardware endpoints replaced by a local stub.
  const resets = [];
  const server = createServer(async (request, response) => {
    const file = request.url.split("?")[0].split("/").pop();
    if (["index.html", "app.js", "styles.css"].includes(file)) {
      response.setHeader("Content-Type", file.endsWith("js") ? "text/javascript" : file.endsWith("css") ? "text/css" : "text/html");
      return response.end(await readFile(new URL(file, import.meta.url)));
    }
    if (request.url === "/api/robot/reset") {
      let body = "";
      for await (const chunk of request) body += chunk;
      resets.push({body: JSON.parse(body), origin: request.headers.origin});
    }
    response.setHeader("Content-Type", "application/json");
    response.end("{}");
  });
  await new Promise(resolve => server.listen(0, "127.0.0.1", resolve));
  try {
    const origin = `http://127.0.0.1:${server.address().port}`;
    await send("Page.navigate", {url: `${origin}/ui/index.html`});
    await evaluate('new Promise(resolve=>{const timer=setInterval(()=>{if(location.protocol==="http:"&&document.readyState==="complete"&&typeof requestRobotReset==="function"){clearInterval(timer);resolve()}},20)})');
    await evaluate('window.confirm=()=>false;requestRobotReset()');
    assert.equal(resets.length, 0);
    await evaluate('window.confirm=()=>true;requestRobotReset()');
    assert.equal(resets.length, 1);
    assert.equal(resets[0].origin, origin);
    assert.equal(resets[0].body.confirmed_estop, true);
    assert.equal(resets[0].body.confirmed_workspace_clear, true);
  } finally {
    server.closeAllConnections();
    await new Promise(resolve => server.close(resolve));
  }
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
