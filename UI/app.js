const views = {
  workbench: "采集作业", datasets: "数据集", devices: "设备管理",
};
const $ = (selector, root = document) => root.querySelector(selector);
const $$ = (selector, root = document) => [...root.querySelectorAll(selector)];
const useApi = location.protocol !== "file:";
const toast = $("#toast");
let toastTimer;
let recordingStarted = 0;
let timerHandle;
let previewTimer;
let pendingDeleteId = "";
let episodeRecords = [];
let picoConnected = false;
let lastPicoStatus = { connected: false };
let lastHardwareStatus = null;
let hardwareRequestError = "";
let collectionError = "";
let robotResetPending = false;
const previewFps = 30;

const delay = (milliseconds) => new Promise((resolve) => setTimeout(resolve, milliseconds));
const escapeHTML = (value) => {
  const node = document.createElement("span");
  node.textContent = String(value ?? "");
  return node.innerHTML;
};

async function api(path, options = {}) {
  const response = await fetch(path, { ...options, headers: { "Content-Type": "application/json", ...options.headers } });
  const body = await response.json().catch(() => ({}));
  if (!response.ok) {
    const message = body.error || `请求失败 (${response.status})`;
    console.error(`[Fieldnote] ${options.method || "GET"} ${path}: ${message}`);
    throw new Error(message);
  }
  return body;
}

function showToast(message) {
  $("span", toast).textContent = message;
  toast.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove("show"), 2800);
}

function renderMonitorErrors() {
  const errors = [];
  if (collectionError) errors.push(`遥操：${collectionError}`);
  if (lastPicoStatus.error) errors.push(`PICO：${lastPicoStatus.error}`);
  if (hardwareRequestError) errors.push(`设备监测：${hardwareRequestError}`);
  if (lastHardwareStatus) {
    if (lastHardwareStatus.marvin.error) errors.push(`机械臂：${lastHardwareStatus.marvin.error}`);
    for (const side of ["left", "right"]) {
      if (lastHardwareStatus.das[side].error) errors.push(`夹爪 ${side}：${lastHardwareStatus.das[side].error}`);
      if (lastHardwareStatus.cameras[side].error) errors.push(`相机 ${side}：${lastHardwareStatus.cameras[side].error}`);
    }
  }
  const panel = $("#monitorErrors");
  panel.classList.toggle("clear", errors.length === 0);
  $("#monitorErrorTitle").textContent = errors.length ? `运行异常 · ${errors.length}` : "实时监督正常";
  $("#monitorErrorTime").textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false });
  $("#monitorErrorList").innerHTML = (errors.length ? errors : ["未发现端口或设备节点异常"])
    .map((message) => `<li>${escapeHTML(message)}</li>`).join("");
}

function openView(name) {
  if (!views[name]) return;
  $$(".view").forEach((view) => view.classList.toggle("active", view.id === `view-${name}`));
  $$('[data-nav]').forEach((item) => item.classList.toggle("active", item.dataset.nav === name && item.classList.contains("nav-item")));
  $("#pageCrumb").textContent = views[name];
  $(".sidebar").classList.remove("open");
  history.replaceState(null, "", `#${name}`);
  window.scrollTo({ top: 0, behavior: "smooth" });
}

function elapsed() {
  const seconds = Math.max(0, Math.floor((Date.now() - recordingStarted) / 1000));
  return [Math.floor(seconds / 3600), Math.floor(seconds / 60) % 60, seconds % 60]
    .map((part) => String(part).padStart(2, "0")).join(":");
}

function syncResetButton(recording = $("#startButton").classList.contains("recording")) {
  $("#resetRobot").disabled = robotResetPending || recording || !lastHardwareStatus?.marvin.connected;
}

function setCameraPreview(active) {
  clearInterval(previewTimer);
  const images = $$(".camera-preview");
  const reset = (image, label) => {
    const feed = image.closest(".camera-feed");
    feed.classList.remove("preview-ready");
    $("[data-preview-state]", feed).textContent = label;
  };
  if (!active || !useApi) {
    images.forEach((image) => {
      image.removeAttribute("src");
      reset(image, "等待帧");
    });
    return;
  }
  const refresh = () => images.forEach((image) => {
    if (image.getAttribute("src") && !image.complete) return;
    image.onload = () => {
      if (!image.naturalWidth) return reset(image, "等待相机");
      const feed = image.closest(".camera-feed");
      feed.classList.add("preview-ready");
      $("[data-preview-state]", feed).textContent = `实时 · ${previewFps} FPS`;
    };
    image.onerror = () => reset(image, "等待相机");
    image.src = `/api/preview/${image.dataset.side}.jpg?t=${Date.now()}`;
  });
  refresh();
  previewTimer = setInterval(refresh, 1000 / previewFps);
}

function setRecording(active, startedAt = Date.now(), notify = true) {
  const button = $("#startButton");
  const live = $("#liveState");
  clearInterval(timerHandle);
  button.disabled = !active && !picoConnected;
  syncResetButton(active);
  setCameraPreview(active);
  if (active) {
    recordingStarted = startedAt;
    button.classList.add("recording");
    $("span", button).textContent = "停止并保存";
    $("use", button).setAttribute("href", "#i-stop");
    $("kbd", button).textContent = "ESC";
    live.classList.add("recording");
    $("span", live).textContent = "RECORDING";
    $("#monitorTitle").textContent = $("[name=task]").value || "未命名任务";
    $("#sessionTimer").textContent = elapsed();
    timerHandle = setInterval(() => $("#sessionTimer").textContent = elapsed(), 250);
    if (notify) showToast("采集进程已启动，正在等待所有数据流就绪");
    return;
  }
  button.classList.remove("recording");
  $("span", button).textContent = "开始采集";
  $("use", button).setAttribute("href", "#i-play");
  $("kbd", button).textContent = "Enter";
  live.classList.remove("recording");
  $("span", live).textContent = "FINALIZING";
  if (notify) showToast("停止信号已发送，正在安全收尾并校验数据");
  setTimeout(() => {
    $("span", live).textContent = "STANDBY";
    $("#monitorTitle").textContent = "等待采集";
  }, 1800);
}

function setPicoStatus(status, data = {}) {
  const connection = $("#picoConnection");
  const health = $("#picoHealth");
  const ports = (data.ports_listening || []).join(" / ");
  const clients = (data.clients || []).join(" / ");
  const service = data.service_ready ? `Robotics Service · ${ports}` : "Robotics Service · 端口未就绪";
  const states = {
    connected: ["PICO 已连接", `${clients} · TCP + Ping 正常`, "在线", service],
    checking: ["正在检测 PICO", "检查 63901 / 60061", "检测中", "Robotics Service · 检查端口"],
    disconnected: ["PICO 未连接", data.error || "服务端口未就绪", "离线", service],
  };
  const state = states[status];
  picoConnected = status === "connected";
  lastPicoStatus = { ...data, connected: picoConnected };
  connection.className = `pico-connection ${status}`;
  $("b", connection).textContent = state[0];
  $("small", connection).textContent = state[1];
  health.className = `health ${status === "connected" ? "good" : status === "checking" ? "checking" : "offline"}`;
  health.textContent = state[2];
  $("#picoService").textContent = state[3];
  $("#picoRate").textContent = status === "connected" ? clients : "—";
  $("#picoLatency").textContent = status === "connected" ? "TCP + Ping" : "—";
  $("#picoStreamRate").textContent = status === "connected" ? "端口在线" : "—";
  $("#picoStreamLatency").textContent = status === "connected" ? clients : "—";
  $("#picoStreamHealth").className = health.className;
  $("#picoStreamHealth").textContent = status === "connected" ? "稳定" : state[2];
  $("#startButton").disabled = status !== "connected" && !$("#startButton").classList.contains("recording");
  updateDeviceSummary();
  renderMonitorErrors();
}

const setHealth = (selector, kind, label) => {
  const element = $(selector);
  element.className = `health ${kind}`;
  element.textContent = label;
};

function updateDeviceSummary() {
  if (!lastHardwareStatus) return;
  const data = lastHardwareStatus;
  const das = Object.values(data.das);
  const cameras = [data.cameras.left, data.cameras.right];
  const devices = Number(lastPicoStatus.connected) + Number(data.marvin.connected)
    + das.filter((item) => item.device_present).length
    + cameras.filter((item) => item.device_present).length;
  $("#deviceOnlineCount").textContent = `${devices} / 6`;
}

function renderHardwareStatus(data) {
  lastHardwareStatus = data;
  const marvin = data.marvin;
  const das = data.das;
  const dasHealthy = das.left.healthy && das.right.healthy;
  const camerasHealthy = data.cameras.left.healthy && data.cameras.right.healthy;
  setHealth("#marvinStreamHealth", marvin.connected ? "good" : "offline", marvin.connected ? "Ping 可达" : "Ping 不通");
  setHealth("#marvinDeviceHealth", marvin.connected ? "good" : "offline", marvin.connected ? "在线" : "离线");
  $("#marvinStreamRate").textContent = marvin.connected ? "可达" : "—";
  $("#marvinStreamAge").textContent = "ICMP Ping";
  $("#marvinControlRate").textContent = marvin.connected ? "可达" : "—";
  $("#marvinStateAge").textContent = marvin.ip;
  $("#marvinFeedback").textContent = marvin.connected ? "Ping 响应正常" : marvin.error;

  setHealth("#dasStreamHealth", dasHealthy ? "good" : "offline", dasHealthy ? "串口就绪" : "串口缺失");
  setHealth("#dasDeviceHealth", dasHealthy && camerasHealthy ? "good" : "offline", dasHealthy && camerasHealthy ? "在线" : "部分缺失");
  $("#dasStreamRate").textContent = `${Number(das.left.device_present) + Number(das.right.device_present)} / 2`;
  $("#dasStreamAge").textContent = "设备节点";
  $("#dasFeedbackRate").textContent = `L ${das.left.device_present ? "已识别" : "缺失"} · R ${das.right.device_present ? "已识别" : "缺失"}`;
  $("#dasDevicePath").textContent = `${das.left.device} · ${das.right.device}`;

  const cameraLabel = (camera) => camera.device_present ? "设备已识别" : "设备缺失";
  setHealth("#visionHealth", camerasHealthy ? "good" : "offline", camerasHealthy ? "设备就绪" : "设备缺失");
  $("#visionRate").textContent = `${Number(data.cameras.left.device_present) + Number(data.cameras.right.device_present)} / 2`;
  $("#visionStreamAge").textContent = "设备节点";
  $("#dasLeftCamera").textContent = cameraLabel(data.cameras.left);
  $("#dasRightCamera").textContent = cameraLabel(data.cameras.right);
  syncResetButton();
  updateDeviceSummary();
  renderMonitorErrors();
}

async function syncHardwareStatus(notify = false) {
  if (!useApi) return;
  try {
    hardwareRequestError = "";
    renderHardwareStatus(await api("/api/devices/hardware"));
    if (notify) showToast("Marvin、DAS 夹爪和相机状态已刷新");
  } catch (error) {
    hardwareRequestError = error.message;
    renderMonitorErrors();
    if (notify) showToast(`设备状态读取失败：${error.message}`);
  }
}

async function checkPico(reconnect = false, silent = false) {
  if (reconnect && $("#startButton").classList.contains("recording")) return showToast("采集中不能重连 PICO，请先停止当前 Episode");
  const button = $("#reconnectPico");
  if (!silent) setPicoStatus("checking");
  if (!silent || reconnect) {
    button.disabled = true;
    button.innerHTML = `<svg><use href="#i-refresh"/></svg>${reconnect ? "正在重连…" : "正在检测…"}`;
  }
  try {
    if (useApi) {
      if (reconnect) {
        await api("/api/devices/pico/reconnect", { method: "POST", body: "{}" });
        for (let attempt = 0; attempt < 3; attempt += 1) {
          await delay(700);
          const status = await api("/api/devices/pico");
          if (status.connected || attempt === 2) {
            setPicoStatus(status.connected ? "connected" : "disconnected", status);
            if (!silent) showToast(status.connected ? "PICO 服务端口已连接" : status.error || "PICO 服务未连接");
            return;
          }
        }
      }
      const status = await api("/api/devices/pico");
      setPicoStatus(status.connected ? "connected" : "disconnected", status);
      if (!silent) showToast(status.connected ? "PICO 服务端口连接正常" : status.error || "PICO 服务未连接");
    } else {
      const message = "UI 后端未启动，请通过 http://127.0.0.1:4173 访问";
      setPicoStatus("disconnected", { error: message });
      if (!silent) showToast(message);
    }
  } catch (error) {
    setPicoStatus("disconnected", { error: error.message });
    if (!silent) showToast(error.message);
  } finally {
    if (!silent || reconnect) {
      button.disabled = false;
      button.innerHTML = '<svg><use href="#i-refresh"/></svg>重新连接 PICO';
    }
  }
}

function syncVisionSettings() {
  const enabled = $("#visionEnabled").checked;
  const resolution = $("#visionResolution");
  resolution.disabled = !enabled;
  $("#visionOptions").classList.toggle("disabled", !enabled);
  $("#cameraGrid").classList.toggle("vision-off", !enabled);
  $$(".camera-format").forEach((item) => item.textContent = enabled ? `${resolution.value.replace("x", " × ")} · 目标 60 Hz` : "已关闭");
  if (lastHardwareStatus) renderHardwareStatus(lastHardwareStatus);
}

async function loadCameraFormats() {
  if (!useApi) return;
  try {
    const data = await api("/api/devices/cameras/formats");
    const resolution = $("#visionResolution");
    const selected = data.formats.some((item) => item.resolution === resolution.value) ? resolution.value : data.formats[0].resolution;
    resolution.replaceChildren(...data.formats.map((item) => new Option(
      `${item.resolution.replace("x", " × ")} · ${data.source === "v4l2" ? "设备" : item.resolution === "640x480" ? "当前" : "SDK"}`,
      item.resolution,
    )));
    resolution.value = selected;
    syncVisionSettings();
  } catch (error) {
    showToast(`相机能力读取失败：${error.message}`);
  }
}

function askDelete(episodeId) {
  pendingDeleteId = episodeId;
  $("#deleteTarget").textContent = episodeId;
  $("#deleteDialog").returnValue = "";
  $("#deleteDialog").showModal();
}

function removeEpisode(episodeId) {
  renderEpisodes(episodeRecords.filter((episode) => episode.id !== episodeId));
}

function formatDuration(seconds) {
  return `${String(Math.floor(seconds / 60)).padStart(2, "0")}:${String(seconds % 60).padStart(2, "0")}`;
}

function formatSize(bytes) {
  return `${(bytes / 1024 ** 3).toFixed(1)} GiB`;
}

const episodeStates = {
  validated: ["已通过", "passed"], degraded: ["待复核", "review"],
  rejected: ["已拒绝", "rejected"], completed: ["已完成", "passed"],
  recording: ["采集中", "review"], aborted: ["已中止", "rejected"],
};

function renderEpisodes(episodes) {
  episodeRecords = episodes;
  const empty = '<tr><td colspan="6">暂无采集数据</td></tr>';
  const rows = episodes.map((episode) => {
    const date = episode.created_at ? new Date(episode.created_at).toLocaleString("zh-CN", { hour12: false }).replaceAll("/", "-") : "—";
    return `<tr data-episode="${escapeHTML(episode.id)}"><td><label class="episode-choice"><input type="checkbox" class="episode-select" aria-label="选择 ${escapeHTML(episode.id)}"><b>EP · ${escapeHTML(episode.id.slice(-8))}</b></label></td><td>${escapeHTML(episode.task)}</td><td>${escapeHTML(date)}</td><td>${episode.modalities.map((item) => `<span class="modality">${escapeHTML(item)}</span>`).join("")}</td><td>${formatDuration(episode.duration_seconds)}</td><td><button class="delete-button" aria-label="删除 Episode"><svg><use href="#i-trash"/></svg></button></td></tr>`;
  });
  $("#datasetRows").innerHTML = rows.join("") || empty;
  $("#recentEpisodeRows").innerHTML = episodes.slice(0, 3).map((episode) => {
    const state = episodeStates[episode.status] || [episode.status, "review"];
    const date = episode.created_at ? new Date(episode.created_at).toLocaleString("zh-CN", { hour12: false }).replaceAll("/", "-") : "—";
    const operator = String(episode.operator ?? "—");
    return `<tr data-episode="${escapeHTML(episode.id)}"><td><b>EP · ${escapeHTML(episode.id.slice(-8))}</b><small>${escapeHTML(date)}</small></td><td>${escapeHTML(episode.task)}</td><td><span class="mini-avatar">${escapeHTML(operator.slice(0, 1).toUpperCase())}</span>${escapeHTML(operator)}</td><td>${formatDuration(episode.duration_seconds)}</td><td>${formatSize(episode.size_bytes)}</td><td><span class="table-status ${state[1]}">${escapeHTML(state[0])}</span></td><td><button class="row-more" aria-label="查看详情"><svg><use href="#i-more"/></svg></button></td></tr>`;
  }).join("") || '<tr><td colspan="7">暂无采集数据</td></tr>';
  $("#datasetCount").textContent = episodes.length;
  const bytes = episodes.reduce((total, item) => total + item.size_bytes, 0);
  $("#datasetSummary").textContent = `共 ${episodes.length} 段 · ${formatSize(bytes)}`;
  syncExportSelection();
}

function showEpisodeDetails(episode) {
  const state = episodeStates[episode.status] || [episode.status, "review"];
  $("#detailId").textContent = episode.id;
  $("#detailStatus").className = `table-status ${state[1]}`;
  $("#detailStatus").textContent = state[0];
  $("#detailTask").textContent = episode.task;
  $("#detailOperator").textContent = episode.operator;
  $("#detailRobot").textContent = episode.robot_model;
  $("#detailModalities").textContent = episode.modalities.join(" · ") || "—";
  $("#detailDuration").textContent = formatDuration(episode.duration_seconds);
  $("#detailSize").textContent = formatSize(episode.size_bytes);
  $("#episodeDialog").showModal();
}

function selectedEpisodeIds() {
  return $$(".episode-select:checked", $("#datasetRows")).map((input) => input.closest("tr").dataset.episode);
}

function syncExportSelection() {
  const visible = $$(".episode-select", $("#datasetRows")).filter((input) => !input.closest("tr").hidden);
  const selected = selectedEpisodeIds();
  const all = $("#selectAllEpisodes");
  $("#exportMcap").disabled = selected.length === 0;
  $("#exportCount").textContent = selected.length;
  all.checked = visible.length > 0 && visible.every((input) => input.checked);
  all.indeterminate = visible.some((input) => input.checked) && !all.checked;
}

function downloadSelectedMcap() {
  const episodeIds = selectedEpisodeIds();
  if (!episodeIds.length) return;
  if (!useApi) return showToast("导出 MCAP 需要启动 UI 后端");
  const url = new URL("/api/exports/mcap", location.href);
  episodeIds.forEach((episodeId) => url.searchParams.append("episode", episodeId));
  const link = document.createElement("a");
  link.href = url;
  link.download = "";
  document.body.append(link);
  link.click();
  link.remove();
  showToast(`正在导出 ${episodeIds.length} 段 Episode`);
}

async function loadEpisodes() {
  if (!useApi) return;
  try {
    const data = await api("/api/episodes");
    renderEpisodes(data.episodes);
  } catch (error) {
    $("#datasetSummary").textContent = "真实数据加载失败";
    showToast(`数据集读取失败：${error.message}`);
  }
}

async function openEpisodeDirectory(episodeId) {
  if (!useApi) return showToast("打开目录需要启动 UI 后端");
  try {
    await api(`/api/episodes/${encodeURIComponent(episodeId)}/open`, { method: "POST", body: "{}" });
    showToast(`${episodeId} 目录打开请求已发送`);
  } catch (error) {
    showToast(error.message);
  }
}

async function startCollection() {
  if (!useApi) return setRecording(true);
  const form = new FormData($("#collectionForm"));
  const payload = {
    task: form.get("task"), operator: form.get("operator"), robot_model: form.get("robot"),
    max_duration: form.get("duration") || null,
    camera_resolution: form.get("camera_resolution") || "640x480",
    camera_fps: 60,
    no_vision: !$("#visionEnabled").checked,
    nsp_lateral: $("#nspLateral").checked,
    confirmed_estop: true, confirmed_joint_mapping: true, confirmed_workspace_clear: true,
  };
  $("#startButton").disabled = true;
  try {
    const job = await api("/api/episodes", { method: "POST", body: JSON.stringify(payload) });
    collectionError = "";
    renderMonitorErrors();
    setRecording(true, job.started_at * 1000);
  } catch (error) {
    $("#startButton").disabled = false;
    showToast(error.message);
  }
}

async function requestRobotReset() {
  if (!useApi) return showToast("机器人复位需要启动 UI 后端");
  robotResetPending = true;
  syncResetButton();
  try {
    await api("/api/robot/reset", { method: "POST", body: "{}" });
    showToast("机器人已复位到初始位");
  } catch (error) {
    showToast(error.message);
  } finally {
    robotResetPending = false;
    syncResetButton();
  }
}

async function stopCollection() {
  if (!useApi) return setRecording(false);
  $("#startButton").disabled = true;
  try {
    await api("/api/episodes/active/stop", { method: "POST", body: "{}" });
    setRecording(false);
  } catch (error) {
    $("#startButton").disabled = false;
    showToast(error.message);
  }
}

async function syncCollectionStatus() {
  if (!useApi) return;
  try {
    const { collection } = await api("/api/status");
    collectionError = collection.status === "failed"
      ? collection.error || `遥操进程异常退出（退出码 ${collection.returncode ?? "未知"}），日志：${collection.log}`
      : "";
    renderMonitorErrors();
    const shownActive = $("#startButton").classList.contains("recording");
    if (collection.active && !shownActive) {
      $("[name=task]").value = collection.task;
      setRecording(true, collection.started_at * 1000, false);
    } else if (!collection.active && shownActive) {
      setRecording(false, Date.now(), false);
      showToast(collection.status === "failed" ? collectionError : "采集已完成并保存");
      loadEpisodes();
    }
  } catch { /* Keep the current state during a transient server failure. */ }
}

$$('[data-nav]').forEach((item) => item.addEventListener("click", (event) => { event.preventDefault(); openView(item.dataset.nav); }));
$("#menuButton").addEventListener("click", () => $(".sidebar").classList.toggle("open"));
$("#runCheck").addEventListener("click", async (event) => {
  if ($("#startButton").classList.contains("recording")) return showToast("采集中不能执行全链路自检");
  const button = event.currentTarget;
  button.disabled = true;
  button.textContent = "检测中…";
  await Promise.all([checkPico(false, true), syncHardwareStatus()]);
  button.innerHTML = '<svg><use href="#i-signal"/></svg>重新检测';
  button.disabled = false;
});
$("#deviceCheck").addEventListener("click", async () => {
  if ($("#startButton").classList.contains("recording")) return showToast("采集中不能执行全链路自检");
  await Promise.all([checkPico(false, true), syncHardwareStatus(true)]);
});
$("#picoConnection").addEventListener("click", () => checkPico(true));
$("#reconnectPico").addEventListener("click", () => checkPico(true));
$("#resetConfig").addEventListener("click", () => { $("#collectionForm").reset(); syncVisionSettings(); showToast("已恢复默认采集配置"); });
$("#visionEnabled").addEventListener("change", syncVisionSettings);
$("#visionResolution").addEventListener("change", syncVisionSettings);

$("#startButton").addEventListener("click", () => {
  if ($("#startButton").classList.contains("recording")) return stopCollection();
  if (!$("[name=task]").value.trim()) { $("[name=task]").focus(); return showToast("请先填写任务名称"); }
  startCollection();
});
$("#resetRobot").addEventListener("click", requestRobotReset);
document.addEventListener("keydown", (event) => {
  if (event.key === "Enter" && event.target.closest("#collectionForm") && !$("#startButton").classList.contains("recording")) {
    event.preventDefault();
    $("#startButton").click();
  }
  if (event.key === "Escape" && $("#startButton").classList.contains("recording") && !$('dialog[open]')) stopCollection();
});
$("#episodeSearch").addEventListener("input", (event) => {
  const query = event.target.value.trim().toLowerCase();
  $$("tr", $("#datasetRows")).forEach((row) => row.hidden = !row.textContent.toLowerCase().includes(query));
  syncExportSelection();
});
$("#selectAllEpisodes").addEventListener("change", (event) => {
  $$(".episode-select", $("#datasetRows")).filter((input) => !input.closest("tr").hidden).forEach((input) => input.checked = event.target.checked);
  syncExportSelection();
});
$("#datasetRows").addEventListener("change", (event) => {
  if (event.target.matches(".episode-select")) syncExportSelection();
});
$("#exportMcap").addEventListener("click", downloadSelectedMcap);
document.addEventListener("click", (event) => {
  const row = event.target.closest("[data-episode]");
  if (!row) return;
  if (event.target.closest(".episode-choice")) return;
  if (event.target.closest(".delete-button")) return askDelete(row.dataset.episode);
  const episode = episodeRecords.find((item) => item.id === row.dataset.episode);
  if (episode) showEpisodeDetails(episode);
});
$("#deleteFromDetail").addEventListener("click", () => {
  const episodeId = $("#detailId").textContent;
  $("#episodeDialog").close();
  askDelete(episodeId);
});
$("#openDirectoryFromDetail").addEventListener("click", () => openEpisodeDirectory($("#detailId").textContent));
$("#deleteDialog").addEventListener("close", async (event) => {
  if (event.currentTarget.returnValue !== "delete") return;
  const episodeId = pendingDeleteId;
  try {
    if (useApi) await api(`/api/episodes/${encodeURIComponent(episodeId)}`, { method: "DELETE", body: JSON.stringify({ confirm: true }) });
    removeEpisode(episodeId);
    showToast(`${episodeId} 已移入回收站`);
  } catch (error) {
    showToast(error.message);
  } finally {
    pendingDeleteId = "";
  }
});

syncVisionSettings();
openView(location.hash.slice(1) || "workbench");
if (useApi) {
  setPicoStatus("checking");
  loadEpisodes();
  Promise.all([checkPico(false, true), syncHardwareStatus(), loadCameraFormats(), syncCollectionStatus()]);
  setInterval(syncCollectionStatus, 2000);
  setInterval(() => checkPico(false, true), 5000);
  setInterval(syncHardwareStatus, 5000);
  document.addEventListener("visibilitychange", () => { if (!document.hidden) checkPico(false, true); });
} else setPicoStatus("disconnected", { error: "UI 后端未启动，请通过 http://127.0.0.1:4173 访问" });
