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
let exportBusy = false;
let pendingDeleteId = "";
let episodeRecords = [];
let picoConnected = false;
let lastPicoStatus = { connected: false };
let lastHardwareStatus = null;
let hardwareRequestError = "";
let deviceError = "";
let collectionError = "";
let robotResetPending = false;
let devicesReady = false;
let previewFps = 30;
let collectionConfigReady = !useApi;
let initialCameraResolution = "";
let previewEnabled = true;
let sessionRecords = [];
let lastCollection = {};
let lastCollectionKey = "";
let reviewSaving = false;
let sessionSaving = false;
const reviewLabels = { success: "成功", failure: "失败", unmarked: "成功（默认）" };
let lastHotkeyEvent = 0;
let hotkeySettingsSync = Promise.resolve();

function syncHotkeySettings() {
  if (!useApi || !devicesActive()) return Promise.resolve();
  // Serialize settings updates so a slow earlier request cannot restore an old Session.
  hotkeySettingsSync = hotkeySettingsSync.catch(() => {}).then(() => api("/api/collection/hotkeys", {
    method: "POST", body: JSON.stringify(collectionPayload()),
  }));
  return hotkeySettingsSync;
}

async function loadSessions(selected) {
  if (!useApi) return;
  sessionRecords = (await api("/api/sessions")).sessions;
  const select = $("#collectionSession");
  const wanted = selected ?? select.value;
  select.replaceChildren(new Option("默认：按日期分组", ""), ...sessionRecords.map(s => new Option(`${s.name} (${s.id})`, s.id)));
  select.value = sessionRecords.some(s => s.id === wanted) ? wanted : "";
  $("#sessionName").value = sessionRecords.find(s => s.id === select.value)?.name || "";
}

async function saveSessionName(create) {
  if (!useApi) return showToast("Session 命名需要启动 UI 后端");
  if (sessionSaving || recordingActive()) return;
  const id = $("#collectionSession").value;
  if (!create && !id) return showToast("请先选择需要重命名的 Session");
  sessionSaving = true;
  syncActionButtons();
  try {
    const saved = await api(create ? "/api/sessions" : `/api/sessions/${encodeURIComponent(id)}/rename`, {
      method: "POST", body: JSON.stringify({ name: $("#sessionName").value.trim() }),
    });
    await loadSessions(saved.id);
    localStorage.setItem("fieldnote-session", saved.id);
    await syncHotkeySettings();
    await loadEpisodes();
    showToast(create ? "Session 已新建，后续采集归入此组" : "Session 名称已更新，数据目录未移动");
  } catch (error) { showToast(error.message); }
  finally { sessionSaving = false; syncActionButtons(); }
}

function reviewCandidates() {
  const records = new Map(episodeRecords.map(e => [`${e.session}/${e.id}`, e]));
  if (lastCollection.episode_id && lastCollection.session && (lastCollection.active || lastCollection.episode_exists)) {
    const key = `${lastCollection.session}/${lastCollection.episode_id}`;
    records.set(key, { ...records.get(key), id: lastCollection.episode_id, session: lastCollection.session,
      review: lastCollection.review, can_review: lastCollection.episode_exists && !lastCollection.active });
  }
  return records;
}

function renderReviewEpisodes(preferred) {
  const select = $("#reviewEpisode");
  const wanted = preferred ?? select.value;
  const records = reviewCandidates();
  select.replaceChildren(...[...records].map(([key, e]) => new Option(`${e.session_name || e.session} / ${e.id}`, key)));
  if (records.has(wanted)) select.value = wanted;
  renderReviewControls();
}

function renderReviewControls() {
  const episode = reviewCandidates().get($("#reviewEpisode").value);
  const result = episode?.review?.result || "unmarked";
  const allowed = episode && episode.can_review !== false && !["starting", "recording", "finalizing"].includes(episode.recording_status);
  $("#reviewResult").textContent = episode ? `人工结果：${reviewLabels[result] || reviewLabels.unmarked}` : "暂无可标注段落";
  $("#reviewEpisode").disabled = reviewSaving;
  $$("[data-review-result]").forEach(button => {
    button.disabled = reviewSaving || !allowed;
    button.setAttribute("aria-pressed", String(button.dataset.reviewResult === result));
  });
}

async function saveEpisodeReview(result) {
  if (!useApi) return showToast("结果标注需要启动 UI 后端");
  const episode = reviewCandidates().get($("#reviewEpisode").value);
  if (!episode || reviewSaving) return;
  reviewSaving = true;
  renderReviewControls();
  try {
    const saved = await api(`/api/episodes/${encodeURIComponent(episode.id)}/review`, {
      method: "POST", body: JSON.stringify({ session: episode.session, result }),
    });
    if (lastCollection.episode_id === episode.id && lastCollection.session === episode.session) lastCollection.review = saved;
    await loadEpisodes();
    showToast(`${episode.id} 已标注：${reviewLabels[result]}`);
  } catch (error) { showToast(error.message); }
  finally { reviewSaving = false; renderReviewControls(); }
}

async function loadCollectionDefaults() {
  const config = await api("/api/collection/config");
  const selectValue = (selector, value) => {
    const select = $(selector);
    if (![...select.options].some((option) => option.value === String(value))) select.add(new Option(String(value), String(value)));
    select.value = String(value);
  };
  selectValue('[name="robot"]', config.robot.model);
  selectValue('[name="duration"]', config.capture.max_duration_s ?? "");
  initialCameraResolution = config.cameras.left.camera_resolution;
  selectValue("#visionResolution", initialCameraResolution);
  $("#visionEnabled").checked = config.capture.vision_enabled;
  $("#nspLateral").checked = config.robot.nsp_lateral;
  previewFps = config.preview.fps;
  previewEnabled = config.preview.enabled;
  collectionConfigReady = true;
  syncVisionSettings();
  syncActionButtons();
}

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
  if (deviceError) errors.push(`设备：${deviceError}`);
  if (collectionError) errors.push(`录制：${collectionError}`);
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

const narrowSidebar = matchMedia("(max-width: 820px)");

function syncSidebar() {
  const sidebar = $("#sidebar");
  const expanded = narrowSidebar.matches ? sidebar.classList.contains("open")
    : !document.body.classList.contains("sidebar-collapsed");
  const button = $("#menuButton");
  button.setAttribute("aria-expanded", String(expanded));
  button.setAttribute("aria-label", expanded ? "收起导航" : "展开导航");
  button.title = button.getAttribute("aria-label");
  if (!expanded && sidebar.contains(document.activeElement)) button.focus();
  sidebar.inert = !expanded;
}

function openView(name) {
  if (!views[name]) return;
  $$(".view").forEach((view) => view.classList.toggle("active", view.id === `view-${name}`));
  $$('[data-nav]').forEach((item) => item.classList.toggle("active", item.dataset.nav === name && item.classList.contains("nav-item")));
  $("#pageCrumb").textContent = views[name];
  $(".sidebar").classList.remove("open");
  syncSidebar();
  history.replaceState(null, "", `#${name}`);
  window.scrollTo({ top: 0, behavior: "smooth" });
}

function elapsed() {
  const seconds = Math.max(0, Math.floor((Date.now() - recordingStarted) / 1000));
  return [Math.floor(seconds / 3600), Math.floor(seconds / 60) % 60, seconds % 60]
    .map((part) => String(part).padStart(2, "0")).join(":");
}

function devicesActive() {
  return $("#deviceButton").classList.contains("devices-active");
}

function recordingActive() {
  return $("#recordButton").classList.contains("recording");
}

function syncActionButtons() {
  $("#deviceButton").disabled = recordingActive() || (!devicesActive() && (!picoConnected || !collectionConfigReady));
  $("#recordButton").disabled = sessionSaving || (!recordingActive() && (!devicesReady || !collectionConfigReady));
  $$(".session-editor input, .session-editor select, .session-editor button").forEach(e => e.disabled = recordingActive() || sessionSaving);
}

function syncResetButton() {
  $("#resetRobot").disabled = robotResetPending || devicesActive() || recordingActive() || !lastHardwareStatus?.marvin.connected;
}

function setDevices(active, ready = active, notify = true) {
  const button = $("#deviceButton");
  devicesReady = active && ready;
  button.classList.toggle("devices-active", active);
  $("span", button).textContent = active ? "停止设备" : "启动设备";
  $("use", button).setAttribute("href", active ? "#i-stop" : "#i-play");
  syncActionButtons();
  syncResetButton();
  if (notify) showToast(active ? "设备启动中，等待数据流就绪" : "设备停止信号已发送");
}

function setCameraPreview(active) {
  clearInterval(previewTimer);
  const images = $$(".camera-preview");
  const reset = (image, label) => {
    const feed = image.closest(".camera-feed");
    feed.classList.remove("preview-ready");
    $("[data-preview-state]", feed).textContent = label;
  };
  if (!active || !useApi || !previewEnabled) {
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
  const button = $("#recordButton");
  const live = $("#liveState");
  clearInterval(timerHandle);
  setCameraPreview(active);
  if (active) {
    recordingStarted = startedAt;
    button.classList.add("recording");
    $("span", button).textContent = "停止并保存";
    $("use", button).setAttribute("href", "#i-stop");
    live.classList.add("recording");
    $("span", live).textContent = "RECORDING";
    $("#monitorTitle").textContent = $("[name=task]").value || "未命名任务";
    $("#sessionTimer").textContent = elapsed();
    timerHandle = setInterval(() => $("#sessionTimer").textContent = elapsed(), 250);
    syncActionButtons();
    syncResetButton();
    if (notify) showToast("录制进程已启动");
    return;
  }
  button.classList.remove("recording");
  $("span", button).textContent = "开始录制";
  $("use", button).setAttribute("href", "#i-play");
  live.classList.remove("recording");
  $("span", live).textContent = "FINALIZING";
  syncActionButtons();
  syncResetButton();
  if (notify) showToast("录制停止信号已发送，正在安全收尾并校验数据");
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
  syncActionButtons();
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
  if (reconnect && (devicesActive() || recordingActive())) return showToast("设备运行中不能重连 PICO");
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
  const selectedSession = $("#sessionFilter").value;
  const sessions = [...new Set(episodes.map((episode) => episode.session).filter(Boolean))].sort().reverse();
  $("#sessionFilter").replaceChildren(new Option("全部 Session", ""), ...sessions.map((session) => new Option(episodes.find(e => e.session === session)?.session_name || session, session)));
  $("#sessionFilter").value = sessions.includes(selectedSession) ? selectedSession : "";
  const empty = '<tr><td colspan="7">暂无采集数据</td></tr>';
  const rows = episodes.map((episode) => {
    const date = episode.created_at ? new Date(episode.created_at).toLocaleString("zh-CN", { hour12: false }).replaceAll("/", "-") : "—";
    return `<tr data-episode="${escapeHTML(episode.id)}" data-session="${escapeHTML(episode.session)}"><td><label class="episode-choice"><input type="checkbox" class="episode-select" aria-label="选择 ${escapeHTML(episode.id)}"><b>EP · ${escapeHTML(episode.id.slice(-8))}</b></label></td><td>${escapeHTML(episode.task)}</td><td>${escapeHTML(date)}</td><td>${episode.modalities.map((item) => `<span class="modality">${escapeHTML(item)}</span>`).join("")}</td><td>${formatDuration(episode.duration_seconds)}</td><td>${reviewLabels[episode.review?.result] || reviewLabels.unmarked}</td><td><button class="delete-button" aria-label="删除 Episode"><svg><use href="#i-trash"/></svg></button></td></tr>`;
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
  filterEpisodes();
  renderReviewEpisodes();
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
  return $$(".episode-select:checked", $("#datasetRows")).filter((input) => !input.closest("tr").hidden).map((input) => input.closest("tr").dataset.episode);
}

function filterEpisodes() {
  const query = $("#episodeSearch").value.trim().toLowerCase();
  const session = $("#sessionFilter").value;
  $$("tr[data-episode]", $("#datasetRows")).forEach((row) => {
    row.hidden = (session && row.dataset.session !== session) || !row.textContent.toLowerCase().includes(query);
  });
  syncExportSelection();
}

function syncExportSelection() {
  const visible = $$(".episode-select", $("#datasetRows")).filter((input) => !input.closest("tr").hidden);
  const selected = selectedEpisodeIds();
  const all = $("#selectAllEpisodes");
  $("#exportMcap").disabled = exportBusy || selected.length === 0;
  $("#exportFormat").disabled = exportBusy;
  $("#exportCount").textContent = selected.length;
  all.checked = visible.length > 0 && visible.every((input) => input.checked);
  all.indeterminate = visible.some((input) => input.checked) && !all.checked;
}

async function exportEpisodeMcaps(episodeIds, directory, request = fetch, variant = "h264") {
  const label = variant === "h264" ? "H.264" : "MJPEG";
  for (const episodeId of episodeIds) {
    showToast(`正在打包 ${episodeId} 的 ${label} MCAP…`);
    const url = new URL("/api/exports/mcap", location.href);
    url.searchParams.set("episode", episodeId);
    url.searchParams.set("format", variant);
    const prepared = await request(new URL("/api/exports/mcap", location.href), {
      method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify({episode: episodeId, format: variant}),
    });
    if (!prepared.ok) {
      const body = await prepared.json().catch(() => ({}));
      throw new Error(body.error || `${episodeId} 打包失败 (${prepared.status})`);
    }
    showToast(`正在下载 ${episodeId} 的 ${label} MCAP`);
    const response = await request(url);
    if (!response.ok) {
      const body = await response.json().catch(() => ({}));
      throw new Error(body.error || `${episodeId} 导出失败 (${response.status})`);
    }
    const filename = response.headers.get("Content-Disposition")?.match(/filename="([^"]+)"/)?.[1] || `${episodeId}.${variant}.mcap`;
    if (filename !== `${episodeId}.${variant}.mcap`) throw new Error(`后端未返回 ${label} MCAP，请重启 UI 后端后重试`);
    const file = await directory.getFileHandle(filename, { create: true });
    await response.body.pipeTo(await file.createWritable());
  }
}

async function downloadSelectedMcap() {
  const episodeIds = selectedEpisodeIds();
  if (!episodeIds.length || exportBusy) return;
  if (!useApi) return showToast("导出 MCAP 需要启动 UI 后端");
  if (!window.showDirectoryPicker) return showToast("批量导出需要使用 Chrome 或 Edge");
  try {
    const directory = await window.showDirectoryPicker({ id: "mcap-export", mode: "readwrite" });
    const variant = $("#exportFormat").value;
    exportBusy = true;
    syncExportSelection();
    $("#exportLabel").textContent = "正在打包并导出…";
    await exportEpisodeMcaps(episodeIds, directory, fetch, variant);
    showToast(`${episodeIds.length} 段 Episode 的 ${variant} MCAP 已依次导出`);
  } catch (error) {
    if (error.name !== "AbortError") showToast(error.message);
  } finally {
    exportBusy = false;
    $("#exportLabel").textContent = "打包并导出";
    syncExportSelection();
    loadEpisodes();
  }
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

function collectionPayload() {
  const form = new FormData($("#collectionForm"));
  return {
    task: form.get("task"), operator: form.get("operator"), robot_model: form.get("robot"),
    session: $("#collectionSession").value || null,
    max_duration: form.get("duration") || null,
    ...(form.get("camera_resolution") !== initialCameraResolution ? { camera_resolution: form.get("camera_resolution") } : {}),
    no_vision: !$("#visionEnabled").checked,
    nsp_lateral: $("#nspLateral").checked,
    confirmed_estop: true, confirmed_joint_mapping: true, confirmed_workspace_clear: true,
  };
}

async function startDevices() {
  if (!useApi) return setDevices(true);
  $("#deviceButton").disabled = true;
  try {
    const job = await api("/api/devices/start", { method: "POST", body: JSON.stringify(collectionPayload()) });
    deviceError = "";
    renderMonitorErrors();
    setDevices(true, job.status === "running");
  } catch (error) {
    syncActionButtons();
    showToast(error.message);
  }
}

async function stopDevices() {
  if (!useApi) return setDevices(false);
  $("#deviceButton").disabled = true;
  try {
    await api("/api/devices/stop", { method: "POST", body: "{}" });
    setDevices(false);
  } catch (error) {
    syncActionButtons();
    showToast(error.message);
  }
}

async function startRecording() {
  if (!useApi) return setRecording(true);
  $("#recordButton").disabled = true;
  try {
    const job = await api("/api/episodes", { method: "POST", body: JSON.stringify(collectionPayload()) });
    lastCollection = job;
    renderReviewEpisodes(`${job.session}/${job.episode_id}`);
    collectionError = "";
    renderMonitorErrors();
    setRecording(true, job.started_at * 1000);
  } catch (error) {
    syncActionButtons();
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

async function stopRecording() {
  if (!useApi) return setRecording(false);
  $("#recordButton").disabled = true;
  try {
    await api("/api/episodes/active/stop", { method: "POST", body: "{}" });
    $("#recordButton span").textContent = "正在保存…";
  } catch (error) {
    syncActionButtons();
    showToast(error.message);
  }
}

async function syncCollectionStatus() {
  if (!useApi) return;
  try {
    const { devices, collection, hotkeys } = await api("/api/status");
    if (hotkeys?.at_ns && hotkeys.at_ns !== lastHotkeyEvent) {
      lastHotkeyEvent = hotkeys.at_ns;
      showToast(hotkeys.message);
    }
    lastCollection = collection;
    const key = `${collection.session}/${collection.episode_id}/${collection.active}/${collection.status}/${collection.episode_exists}`;
    if (key !== lastCollectionKey) {
      lastCollectionKey = key;
      await loadEpisodes();
      renderReviewEpisodes(`${collection.session}/${collection.episode_id}`);
      if (!collection.active && collection.episode_id) await loadSessions();
    } else renderReviewControls();
    deviceError = devices.status === "failed"
      ? devices.error || `设备进程异常退出（退出码 ${devices.returncode ?? "未知"}），日志：${devices.log}`
      : "";
    collectionError = collection.status === "failed"
      ? collection.error || `录制进程异常退出（退出码 ${collection.returncode ?? "未知"}），日志：${collection.log}`
      : "";
    renderMonitorErrors();
    const shownDevices = devicesActive();
    if (devices.active) {
      setDevices(true, devices.status === "running", false);
    } else if (shownDevices) {
      setDevices(false, false, false);
      showToast(devices.status === "failed" ? deviceError : "设备已停止");
    }
    const shownActive = recordingActive();
    if (collection.active && !shownActive) {
      $("[name=task]").value = collection.task;
      setRecording(true, collection.started_at * 1000, false);
    } else if (!collection.active && shownActive) {
      setRecording(false, Date.now(), false);
      showToast(collection.status === "failed" ? collectionError : "录制已完成并保存");
      loadEpisodes();
    }
    if (collection.active) {
      $("#recordButton").disabled = collection.status !== "running";
      $("#recordButton span").textContent = collection.status === "running" ? "结束录制"
        : ["stopping", "saving"].includes(collection.status) ? "正在保存…" : "启动中…";
      if (["stopping", "saving"].includes(collection.status)) {
        clearInterval(timerHandle);
        setCameraPreview(false);
        $("#liveState span").textContent = "正在保存";
      }
    }
  } catch { /* Keep the current state during a transient server failure. */ }
}

$$('[data-nav]').forEach((item) => item.addEventListener("click", (event) => { event.preventDefault(); openView(item.dataset.nav); }));
$("#menuButton").addEventListener("click", () => {
  if (narrowSidebar.matches) $("#sidebar").classList.toggle("open");
  else document.body.classList.toggle("sidebar-collapsed");
  syncSidebar();
});
narrowSidebar.addEventListener("change", () => {
  $("#sidebar").classList.remove("open");
  syncSidebar();
});
$("#runCheck").addEventListener("click", async (event) => {
  if (recordingActive()) return showToast("采集中不能执行全链路自检");
  const button = event.currentTarget;
  button.disabled = true;
  button.textContent = "检测中…";
  await Promise.all([checkPico(false, true), syncHardwareStatus()]);
  button.innerHTML = '<svg><use href="#i-signal"/></svg>重新检测';
  button.disabled = false;
});
$("#deviceCheck").addEventListener("click", async () => {
  if (recordingActive()) return showToast("采集中不能执行全链路自检");
  await Promise.all([checkPico(false, true), syncHardwareStatus(true)]);
});
$("#picoConnection").addEventListener("click", () => checkPico(true));
$("#reconnectPico").addEventListener("click", () => checkPico(true));
$("#resetConfig").addEventListener("click", async () => {
  if (devicesActive() || recordingActive()) return showToast("请先停止设备再重置配置");
  $("#collectionForm").reset();
  try { if (useApi) await loadCollectionDefaults(); syncVisionSettings(); showToast("已恢复默认采集配置"); }
  catch (error) { showToast(error.message); }
});
$("#visionEnabled").addEventListener("change", syncVisionSettings);
$("#visionResolution").addEventListener("change", syncVisionSettings);

$("#deviceButton").addEventListener("click", () => {
  if (devicesActive()) return stopDevices();
  startDevices();
});
$("#recordButton").addEventListener("click", () => {
  if (recordingActive()) return stopRecording();
  if (!$("[name=task]").value.trim()) { $("[name=task]").focus(); return showToast("请先填写任务名称"); }
  startRecording();
});
$("#resetRobot").addEventListener("click", requestRobotReset);
document.addEventListener("keydown", (event) => {
  if (event.key === "Enter" && event.target.closest(".session-editor")) { event.preventDefault(); return; }
  if (event.key === "Enter" && event.target.closest("#collectionForm") && !recordingActive()) {
    event.preventDefault();
    $("#recordButton").click();
  }
  if (event.key === "Escape" && recordingActive() && !$('dialog[open]')) stopRecording();
});
$("#episodeSearch").addEventListener("input", filterEpisodes);
$("#createSession").addEventListener("click", () => saveSessionName(true));
$("#renameSession").addEventListener("click", () => saveSessionName(false));
$("#collectionSession").addEventListener("change", () => {
  $("#sessionName").value = sessionRecords.find(s => s.id === $("#collectionSession").value)?.name || "";
  localStorage.setItem("fieldnote-session", $("#collectionSession").value);
});
$("#reviewEpisode").addEventListener("change", renderReviewControls);
$("#collectionForm").addEventListener("input", () => syncHotkeySettings().catch(error => showToast(`手柄参数同步失败：${error.message}`)));
$("#collectionForm").addEventListener("change", () => syncHotkeySettings().catch(error => showToast(`手柄参数同步失败：${error.message}`)));
$$("[data-review-result]").forEach(button => button.addEventListener("click", () => saveEpisodeReview(button.dataset.reviewResult)));
$("#sessionFilter").addEventListener("change", filterEpisodes);
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
  loadSessions(localStorage.getItem("fieldnote-session") || "").catch(error => showToast(error.message));
  loadEpisodes();
  loadCollectionDefaults().then(() => Promise.all([checkPico(false, true), syncHardwareStatus(), loadCameraFormats(), syncCollectionStatus()]))
    .catch((error) => showToast(`配置加载失败：${error.message}`));
  setInterval(syncCollectionStatus, 2000);
  setInterval(() => checkPico(false, true), 5000);
  setInterval(syncHardwareStatus, 5000);
  document.addEventListener("visibilitychange", () => { if (!document.hidden) checkPico(false, true); });
} else setPicoStatus("disconnected", { error: "UI 后端未启动，请通过 http://127.0.0.1:4173 访问" });
