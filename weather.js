// ==================== 天气数据定义 ====================
export const CARLA_PRESETS = {
    ClearNoon: {  desc: "晴朗正午", params: { cloudiness: 0, precipitation: 0, precipitation_deposits: 0, wind_intensity: 0, fog_density: 0, fog_distance: 0, fog_falloff: 0, wetness: 0, sun_azimuth_angle: 120, sun_altitude_angle: 75 } },
    CloudyNoon: {  desc: "多云正午", params: { cloudiness: 70, precipitation: 0, precipitation_deposits: 0, wind_intensity: 20, fog_density: 10, fog_distance: 50, fog_falloff: 1, wetness: 0, sun_azimuth_angle: 130, sun_altitude_angle: 68 } },
    WetNoon: {  desc: "潮湿正午", params: { cloudiness: 40, precipitation: 20, precipitation_deposits: 15, wind_intensity: 25, fog_density: 15, fog_distance: 40, fog_falloff: 0.8, wetness: 50, sun_azimuth_angle: 125, sun_altitude_angle: 70 } },
    WetCloudyNoon: {  desc: "湿阴天午间", params: { cloudiness: 90, precipitation: 25, precipitation_deposits: 20, wind_intensity: 35, fog_density: 30, fog_distance: 40, fog_falloff: 0.8, wetness: 65, sun_azimuth_angle: 140, sun_altitude_angle: 62 } },
    MidRainyNoon: {  desc: "中雨正午", params: { cloudiness: 85, precipitation: 50, precipitation_deposits: 45, wind_intensity: 55, fog_density: 45, fog_distance: 25, fog_falloff: 0.6, wetness: 80, sun_azimuth_angle: 150, sun_altitude_angle: 65 } },
    HardRainNoon: {  desc: "暴雨正午", params: { cloudiness: 95, precipitation: 85, precipitation_deposits: 80, wind_intensity: 80, fog_density: 65, fog_distance: 15, fog_falloff: 0.4, wetness: 95, sun_azimuth_angle: 160, sun_altitude_angle: 60 } },
    SoftRainNoon: {  desc: "小雨正午", params: { cloudiness: 75, precipitation: 30, precipitation_deposits: 25, wind_intensity: 40, fog_density: 25, fog_distance: 35, fog_falloff: 0.7, wetness: 60, sun_azimuth_angle: 145, sun_altitude_angle: 67 } },
    ClearSunset: {  desc: "晴朗日落", params: { cloudiness: 0, precipitation: 0, precipitation_deposits: 0, wind_intensity: 0, fog_density: 0, fog_distance: 0, fog_falloff: 0, wetness: 0, sun_azimuth_angle: 280, sun_altitude_angle: 15 } },
    CloudySunset: {  desc: "多云日落", params: { cloudiness: 75, precipitation: 0, precipitation_deposits: 0, wind_intensity: 25, fog_density: 15, fog_distance: 45, fog_falloff: 1, wetness: 0, sun_azimuth_angle: 270, sun_altitude_angle: 12 } },
    WetSunset: {  desc: "潮湿日落", params: { cloudiness: 50, precipitation: 15, precipitation_deposits: 10, wind_intensity: 20, fog_density: 10, fog_distance: 35, fog_falloff: 0.7, wetness: 40, sun_azimuth_angle: 275, sun_altitude_angle: 10 } },
    WetCloudySunset: {  desc: "湿阴天日落", params: { cloudiness: 85, precipitation: 20, precipitation_deposits: 15, wind_intensity: 30, fog_density: 25, fog_distance: 38, fog_falloff: 0.75, wetness: 55, sun_azimuth_angle: 265, sun_altitude_angle: 8 } },
    MidRainSunset: {  desc: "中雨日落", params: { cloudiness: 88, precipitation: 55, precipitation_deposits: 50, wind_intensity: 60, fog_density: 48, fog_distance: 22, fog_falloff: 0.58, wetness: 82, sun_azimuth_angle: 260, sun_altitude_angle: 5 } },
    HardRainSunset: {  desc: "暴雨日落", params: { cloudiness: 96, precipitation: 88, precipitation_deposits: 82, wind_intensity: 85, fog_density: 68, fog_distance: 12, fog_falloff: 0.38, wetness: 96, sun_azimuth_angle: 255, sun_altitude_angle: 3 } },
    SoftRainSunset: { desc: "小雨日落", params: { cloudiness: 78, precipitation: 35, precipitation_deposits: 28, wind_intensity: 42, fog_density: 28, fog_distance: 32, fog_falloff: 0.68, wetness: 65, sun_azimuth_angle: 268, sun_altitude_angle: 6 } }
};

export const EXPERT_BASE_PARAMS = [
    { id: "cloudiness", label: "云量", min: 0, max: 100, step: 1, default: 0 },
    { id: "precipitation", label: "降雨强度", min: 0, max: 100, step: 1, default: 0 },
    { id: "precipitation_deposits", label: "路面积水", min: 0, max: 100, step: 1, default: 0 },
    { id: "wind_intensity", label: "风力强度", min: 0, max: 100, step: 1, default: 0 },
    { id: "fog_density", label: "雾浓度", min: 0, max: 100, step: 1, default: 0 },
    { id: "wetness", label: "路面湿度", min: 0, max: 100, step: 1, default: 0 },
    { id: "sun_azimuth_angle", label: "太阳方位角", min: 0, max: 360, step: 1, default: 0 },
    { id: "sun_altitude_angle", label: "太阳高度角", min: -90, max: 90, step: 1, default: 0 },
    { id: "fog_distance", label: "雾起始距离(m)", min: 0, max: 200, step: 1, default: 50 },
    { id: "fog_falloff", label: "雾密度衰减", min: 0, max: 3, step: 0.1, default: 0.8 }
];

import { state, showNotification } from './core.js';
// ==================== 辅助函数 ====================
function getExpertFullParams() {
    return {...state.expertParams};
}

function getCurrentBaseParams() {
    // 始终返回专家参数（因为现在静态天气下两者同时可见，专家参数为最终状态）
    return getExpertFullParams();
}

// ==================== 天气序列生成 ====================
export function generateWeatherSequence() {
    let base = getCurrentBaseParams();
    let duration = state.collectDuration;
    let fps = state.sampleFps;
    let step = 1 / fps;
    let totalFrames = duration*fps;
    let frames = [];

    for (let i = 0; i < totalFrames; i++) {
        let t = Math.min(i * step, duration);
        let weatherParams = { ...base };
        frames.push({
            timestamp: t,
            weatherParams: weatherParams
        });
    }

    const activeSensors = Object.entries(state.sensorFullConfig).filter(([, v]) => v.enabled).map(([id]) => id);
    const sensorsFullConfig = JSON.parse(JSON.stringify(state.sensorFullConfig));

    return {
        mode: "expert", // 统一为专家模式，因为滑块始终有效
        durationSec: duration,
        fps: fps,
        totalFrames: frames.length,
        frames,
        activeSensors,
        sensorsFullConfig,
        carlaMap: state.currentMap,
        traffic: {
            vehiclesEnabled: state.vehiclesEnabled,
            pedestrianEnabled: state.pedestrianEnabled,
            vehicleCount: state.vehicleCount,
            pedestrianCount: state.pedestrianCount
        },
        generatedAt: new Date().toISOString(),
        savePath: state.currentSavePath
    };
}

// ==================== UI 渲染 ====================
export function updatePreview() {
    const weatherNameSpan = document.getElementById("weatherName");
    const statusSpan = document.getElementById("statusBar");

    // 从当前专家参数生成预览（始终使用专家参数，因为预设会同步更新滑块）
    if (weatherNameSpan) {
        if (state.currentPresetId && CARLA_PRESETS[state.currentPresetId]) {
            weatherNameSpan.innerText = CARLA_PRESETS[state.currentPresetId].desc;
        } else {
            weatherNameSpan.innerText = "自定义天气";
        }
    }
    if (statusSpan) {
    statusSpan.innerHTML =
        `🔧 静态天气 · 车辆:${state.vehiclesEnabled ? state.vehicleCount + "辆" : "关闭"} 行人:${state.pedestrianEnabled ? state.pedestrianCount + "人" : "关闭"} · ${state.collectDuration}s/${state.sampleFps}fps`;
    }
}

function renderExpertUI() {
    const container = document.getElementById("expertParamsContainer");
    if (!container){
        return;
    }
    container.innerHTML = "";

    const allParams = EXPERT_BASE_PARAMS;

    allParams.forEach(def => {
        const row = document.createElement("div");
        row.className = "param-row";

        const currentValue = state.expertParams[def.id] !== undefined ? state.expertParams[def.id] : def.default;

        const displayVal = def.id.includes("angle") ? Math.round(currentValue) : parseFloat(currentValue.toFixed(1));

        row.innerHTML = `
            <div class="param-label">${def.label}</div>
            <input type="range" id="exp_${def.id}" min="${def.min}" max="${def.max}" step="${def.step}" value="${currentValue}">
            <span class="param-value" id="exp_${def.id}_val">${displayVal}</span>
        `;

        const slider = row.querySelector("input");
        const span = row.querySelector(".param-value");

        slider.addEventListener("input", (e) => {
            let v = parseFloat(e.target.value);
            // 清除预设高亮
            document.querySelectorAll('.preset-card').forEach(c => c.classList.remove('selected'));
            state.currentPresetId = null;

            // 更新对应的 state
            state.expertParams[def.id] = v;

            const display = def.id.includes("angle") ? Math.round(v) : v.toFixed(1);
            span.innerText = display;
            updatePreview();
        });

        container.appendChild(row);
    });
}

function renderPresetsUI() {
    const container = document.getElementById("presetGridContainer");
    if (!container) return;

    const grid = document.createElement("div");
    grid.className = "preset-grid";

    Object.keys(CARLA_PRESETS).forEach(pid => {
        const p = CARLA_PRESETS[pid];
        const card = document.createElement("div");
        card.className = "preset-card";
        card.dataset.presetId = pid;
        card.innerHTML = `
            <div style="font-size: 13px; font-weight: 700; margin-top: 4px;">${p.desc}</div>
            <div style="font-size: 10px; color: #7e8b9f; margin-top: 2px;">${pid}</div>
        `;

        card.addEventListener("click", () => {
            // 清除所有高亮
            document.querySelectorAll('.preset-card').forEach(c => c.classList.remove('selected'));
            card.classList.add('selected');
            applyPreset(pid);
            showNotification(`切换到预设: ${p.desc}`, "info");
        });

        grid.appendChild(card);
    });

    container.innerHTML = "";
    container.appendChild(grid);
}

function applyPreset(presetId) {
    const preset = CARLA_PRESETS[presetId];
    if (!preset) return;

    // 记录当前预设ID
    state.currentPresetId = presetId;

    const params = preset.params;

    // 更新 state
    EXPERT_BASE_PARAMS.forEach(def => {
        if (params[def.id] !== undefined) {
            state.expertParams[def.id] = params[def.id];
        }
    });

    // 更新所有滑块的值
    document.querySelectorAll('#expertParamsContainer input[type="range"]').forEach(slider => {
        const id = slider.id.replace('exp_', '');
        let val = state.expertParams[id];

        if (val !== undefined) {
            slider.value = val;
            const span = document.getElementById(`exp_${id}_val`);
            if (span) {
                const display = id.includes("angle") ? Math.round(val) : parseFloat(val.toFixed(1));
                span.innerText = display;
            }
        }
    });

    updatePreview();
}

// ==================== 标签页切换 ====================
function bindWeatherModeSwitch() {
    const modeBtns = document.querySelectorAll('#weatherModeSwitch .mode-btn');
    const staticContent = document.getElementById('staticWeatherContent');
    const dynamicContent = document.getElementById('dynamicWeatherContent');

    modeBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            const mode = btn.getAttribute('data-mode');

            // 更新按钮状态
            modeBtns.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            // 切换内容显示
            if (mode === 'static') {
                staticContent.style.display = 'block';
                dynamicContent.style.display = 'none';
            } else {
                staticContent.style.display = 'none';
                dynamicContent.style.display = 'block';
            }
        });
    });
}

// ==================== 初始化 ====================
export function initWeather() {
    // 1. 绑定静态/动态标签页切换
    bindWeatherModeSwitch();

    // 2. 确保默认显示静态天气
    const staticContent = document.getElementById('staticWeatherContent');
    const dynamicContent = document.getElementById('dynamicWeatherContent');
    if (staticContent) staticContent.style.display = 'block';
    if (dynamicContent) dynamicContent.style.display = 'none';

    // 3. 确保静态标签页激活
    const staticBtn = document.querySelector('#weatherModeSwitch .mode-btn[data-mode="static"]');
    if (staticBtn) staticBtn.classList.add('active');

    // 4. 渲染专家滑块和预设卡片
    renderExpertUI();
    renderPresetsUI();

    // 5. 默认选中 ClearNoon 并高亮
    const defaultPresetId = 'ClearNoon';
    applyPreset(defaultPresetId);
    const defaultCard = document.querySelector(`.preset-card[data-preset-id="${defaultPresetId}"]`);
    if (defaultCard) defaultCard.classList.add('selected');

    // 6. 更新预览
    updatePreview();
    initDynamicWeather();
}

// ==================== 动态天气编辑器 ====================

// 模拟数据
const DW_MOCK_TRACKS = [
    { id: 'rain', name: '降雨', color: '#3b82f6', visible: true, active: true },
    { id: 'wind', name: '风力', color: '#f59e0b', visible: true, active: false },
    { id: 'cloud', name: '云量', color: '#8b5cf6', visible: false, active: false },
    { id: 'fog', name: '雾浓度', color: '#ec4899', visible: true, active: false },
];

// 锚点模拟数据（降雨轨道）
const MOCK_ANCHORS_RAIN = [
    { t: 0, val: 0, continuity: 'C2', handleIn: null, handleOut: null },
    { t: 5, val: 78, continuity: 'C0', handleIn: [3, 45], handleOut: [8, 78] },
    { t: 10, val: 40, continuity: 'C1', handleIn: [7, 40], handleOut: [13, 40] },
    { t: 15, val: 90, continuity: 'C2', handleIn: null, handleOut: null },
    { t: 20, val: 55, continuity: 'C0', handleIn: [17, 55], handleOut: [23, 55] },
    { t: 30, val: 30, continuity: 'C2', handleIn: null, handleOut: null },
];

let dwState = {
    tracks: JSON.parse(JSON.stringify(DW_MOCK_TRACKS)),
    anchors: {
        rain: JSON.parse(JSON.stringify(MOCK_ANCHORS_RAIN)),
        wind: [{ t: 0, val: 20 }, { t: 15, val: 70 }, { t: 30, val: 40 }],
        cloud: [{ t: 0, val: 10 }, { t: 30, val: 60 }],
        fog: [{ t: 0, val: 5 }, { t: 10, val: 60 }, { t: 30, val: 10 }],
    },
    selectedTrackId: 'rain',
    selectedAnchorIndex: 1, // 选中 @5s 锚点
    cursorTime: 10,
    duration: 30,
};

// ---- 初始化动态天气编辑器 ----
export function initDynamicWeather() {
    const container = document.getElementById('dynamicWeatherContent');
    if (!container) return;

    // 1. 渲染轨道列表
    renderTrackList();

    // 2. 渲染画布
    renderCanvas();

    // 3. 渲染属性面板
    renderPropertyPanel();

    // 4. 渲染标尺
    renderRuler();

    // 5. 绑定顶部工具栏事件（占位）
    document.getElementById('dwPreviewBtn')?.addEventListener('click', () => {
        showNotification('预览功能开发中', 'info');
    });

    // 6. 切换模式（曲线/表格）
    document.querySelectorAll('.dw-mode-btn').forEach(btn => {
        btn.addEventListener('click', function() {
            document.querySelectorAll('.dw-mode-btn').forEach(b => b.classList.remove('active'));
            this.classList.add('active');
            showNotification(this.dataset.mode === 'timeline' ? '切换到曲线视图' : '切换到表格视图', 'info');
        });
    });

    // 7. 添加轨道（占位）
    document.getElementById('dwAddTrackBtn')?.addEventListener('click', () => {
        showNotification('添加新曲线功能开发中', 'info');
    });
}

// ---- 渲染轨道列表 ----
function renderTrackList() {
    const list = document.getElementById('dwTrackList');
    if (!list) return;
    list.innerHTML = '';
    dwState.tracks.forEach(track => {
        const div = document.createElement('div');
        div.className = `dw-track-item${track.active ? ' active' : ''}`;
        div.dataset.trackId = track.id;
        div.innerHTML = `
            <div class="dw-track-color" style="background:${track.color};"></div>
            <div class="dw-track-name">${track.name}</div>
            <div class="dw-track-vis${track.visible ? ' on' : ''}">👁️</div>
        `;
        // 点击选中轨道
        div.addEventListener('click', () => {
            dwState.tracks.forEach(t => t.active = false);
            track.active = true;
            dwState.selectedTrackId = track.id;
            renderTrackList();
            renderCanvas();
            renderPropertyPanel();
        });
        // 点击眼睛切换显隐
        const vis = div.querySelector('.dw-track-vis');
        vis.addEventListener('click', (e) => {
            e.stopPropagation();
            track.visible = !track.visible;
            renderTrackList();
            renderCanvas();
        });
        list.appendChild(div);
    });
}

// ---- 渲染标尺 ----
function renderRuler() {
    const ruler = document.getElementById('dwRuler');
    if (!ruler) return;
    ruler.innerHTML = '';
    const total = dwState.duration;
    const step = 5;
    for (let i = 0; i <= total; i += step) {
        const pos = (i / total) * 100;
        const tick = document.createElement('div');
        tick.className = 'dw-ruler-tick';
        tick.style.left = pos + '%';
        ruler.appendChild(tick);
        const mark = document.createElement('div');
        mark.className = 'dw-ruler-mark';
        mark.style.left = pos + '%';
        mark.textContent = i + 's';
        ruler.appendChild(mark);
    }
}

// ---- 渲染画布 ----
function renderCanvas() {
    const svg = document.getElementById('dwSvg');
    if (!svg) return;
    const width = 800, height = 320;
    const margin = { left: 40, right: 20, top: 20, bottom: 20 };
    const plotWidth = width - margin.left - margin.right;
    const plotHeight = height - margin.top - margin.bottom;
    const total = dwState.duration;

    // 映射函数
    const xScale = (t) => margin.left + (t / total) * plotWidth;
    const yScale = (val) => margin.top + plotHeight - (val / 100) * plotHeight;

    // 构建 SVG 内容
    let html = '';

    // 1. 网格
    for (let v = 0; v <= 100; v += 25) {
        const y = yScale(v);
        const cls = (v === 0 || v === 100) ? 'dw-grid-solid' : 'dw-grid-line';
        html += `<line x1="${margin.left}" y1="${y}" x2="${width - margin.right}" y2="${y}" class="${cls}" />`;
    }
    for (let t = 0; t <= total; t += 5) {
        const x = xScale(t);
        html += `<line x1="${x}" y1="${margin.top}" x2="${x}" y2="${height - margin.bottom}" class="dw-grid-line" />`;
    }

    // 2. 绘制所有可见轨道曲线
    dwState.tracks.forEach(track => {
        if (!track.visible) return;
        const anchors = dwState.anchors[track.id] || [];
        if (anchors.length < 2) return;

        // 构建贝塞尔路径（简化：仅线性插值展示，非真实贝塞尔）
        let pathD = '';
        for (let i = 0; i < anchors.length - 1; i++) {
            const a = anchors[i];
            const b = anchors[i + 1];
            const x1 = xScale(a.t), y1 = yScale(a.val);
            const x2 = xScale(b.t), y2 = yScale(b.val);
            // 使用三次贝塞尔（带手柄模拟）
            let cp1x = x1 + (x2 - x1) * 0.33;
            let cp1y = y1;
            let cp2x = x2 - (x2 - x1) * 0.33;
            let cp2y = y2;
            // 如果锚点有手柄数据（仅针对rain）
            if (track.id === 'rain' && a.handleOut) {
                cp1x = xScale(a.handleOut[0]);
                cp1y = yScale(a.handleOut[1]);
            }
            if (track.id === 'rain' && b.handleIn) {
                cp2x = xScale(b.handleIn[0]);
                cp2y = yScale(b.handleIn[1]);
            }
            if (i === 0) pathD += `M ${x1} ${y1}`;
            pathD += ` C ${cp1x} ${cp1y}, ${cp2x} ${cp2y}, ${x2} ${y2}`;
        }
        html += `<path d="${pathD}" class="dw-curve" style="stroke:${track.color}; opacity:${track.id === dwState.selectedTrackId ? 1 : 0.7};" />`;

        // 绘制锚点
        anchors.forEach((a, idx) => {
            const cx = xScale(a.t), cy = yScale(a.val);
            const isSelected = (track.id === dwState.selectedTrackId && idx === dwState.selectedAnchorIndex);
            let shape = '';
            let fill = track.color;
            let stroke = '#1e293b';
            let extra = '';

            // 根据连续性决定形状
            const cont = a.continuity || 'C2';
            if (cont === 'C0') {
                // 菱形
                const size = isSelected ? 8 : 6;
                const pts = `${cx},${cy-size} ${cx+size},${cy} ${cx},${cy+size} ${cx-size},${cy}`;
                shape = `<polygon points="${pts}" fill="${fill}" stroke="${stroke}" stroke-width="2" />`;
                // 显示手柄
                if (a.handleOut) {
                    const hx = xScale(a.handleOut[0]), hy = yScale(a.handleOut[1]);
                    html += `<line x1="${cx}" y1="${cy}" x2="${hx}" y2="${hy}" class="dw-handle-line" />`;
                    html += `<circle cx="${hx}" cy="${hy}" r="4" class="dw-handle-point" />`;
                }
                if (a.handleIn) {
                    const hx = xScale(a.handleIn[0]), hy = yScale(a.handleIn[1]);
                    html += `<line x1="${cx}" y1="${cy}" x2="${hx}" y2="${hy}" class="dw-handle-line" />`;
                    html += `<circle cx="${hx}" cy="${hy}" r="4" class="dw-handle-point" />`;
                }
            } else if (cont === 'C1') {
                // 圆形
                const r = isSelected ? 8 : 6;
                shape = `<circle cx="${cx}" cy="${cy}" r="${r}" fill="${fill}" stroke="${stroke}" stroke-width="2" />`;
                // 显示联动手柄（仅出段示例）
                if (a.handleOut) {
                    const hx = xScale(a.handleOut[0]), hy = yScale(a.handleOut[1]);
                    html += `<line x1="${cx}" y1="${cy}" x2="${hx}" y2="${hy}" class="dw-handle-line" />`;
                    html += `<circle cx="${hx}" cy="${hy}" r="4" class="dw-handle-point" />`;
                }
            } else {
                // C2: 三角形
                const size = isSelected ? 8 : 6;
                const pts = `${cx},${cy-size} ${cx+size},${cy+size} ${cx-size},${cy+size}`;
                shape = `<polygon points="${pts}" fill="${fill}" stroke="${stroke}" stroke-width="2" />`;
            }
            html += `<g class="dw-anchor-point" data-track="${track.id}" data-index="${idx}">${shape}</g>`;
            // 显示时间标签（仅选中的锚点）
            if (isSelected) {
                html += `<text x="${cx}" y="${cy + 20}" font-size="8" fill="#94a3b8" text-anchor="middle">${a.t}s</text>`;
            }
        });
    });

    // 3. 当前时间指示器
    const cursorX = xScale(dwState.cursorTime);
    html += `<line x1="${cursorX}" y1="${margin.top}" x2="${cursorX}" y2="${height - margin.bottom}" class="dw-current-indicator" />`;

    svg.innerHTML = html;

    // 绑定锚点点击事件
    svg.querySelectorAll('.dw-anchor-point').forEach(el => {
        el.addEventListener('click', function() {
            const trackId = this.dataset.track;
            const idx = parseInt(this.dataset.index);
            dwState.selectedTrackId = trackId;
            dwState.selectedAnchorIndex = idx;
            // 更新轨道激活状态
            dwState.tracks.forEach(t => t.active = (t.id === trackId));
            renderTrackList();
            renderPropertyPanel();
            renderCanvas(); // 刷新高亮
        });
    });
}

// ---- 渲染属性面板 ----
function renderPropertyPanel() {
    const panel = document.getElementById('dwRightPanel');
    if (!panel) return;

    const trackId = dwState.selectedTrackId;
    const anchors = dwState.anchors[trackId] || [];
    const idx = dwState.selectedAnchorIndex;
    const anchor = (idx >= 0 && idx < anchors.length) ? anchors[idx] : null;

    if (!anchor) {
        panel.innerHTML = `<div class="dw-property-placeholder">点击锚点查看属性</div>`;
        return;
    }

    const track = dwState.tracks.find(t => t.id === trackId);
    const color = track ? track.color : '#3b82f6';
    const cont = anchor.continuity || 'C2';
    const contLabel = cont === 'C0' ? 'C0 · 自由手柄' : cont === 'C1' ? 'C1 · 统一手柄' : 'C2 · 全自动平滑';
    const contBadge = `dw-continuity-badge ${cont.toLowerCase()}`;

    let html = `
        <div class="dw-prop-group">
            <div class="dw-prop-title">🔷 选中锚点 (${track ? track.name : ''} @ ${anchor.t}s)</div>
            <div class="dw-param-row">
                <label>时间 (t)</label>
                <span class="dw-param-value">${anchor.t.toFixed(1)} s</span>
            </div>
            <div class="dw-param-row">
                <label>数值 (val)</label>
                <span class="dw-param-value">${anchor.val.toFixed(1)}</span>
            </div>
            <div class="dw-param-row">
                <label>连续性</label>
                <select id="dwContinuitySelect">
                    <option value="C0" ${cont === 'C0' ? 'selected' : ''}>C0 · 自由手柄</option>
                    <option value="C1" ${cont === 'C1' ? 'selected' : ''}>C1 · 统一手柄</option>
                    <option value="C2" ${cont === 'C2' ? 'selected' : ''}>C2 · 全自动平滑</option>
                </select>
            </div>
            <div style="display:flex; gap:8px; margin-top:8px;">
                <span class="${contBadge}">${contLabel}</span>
                <span style="font-size:10px; color:#64748b;">
                    ${cont === 'C0' ? '手柄独立调节' : cont === 'C1' ? '手柄联动' : '全自动推导'}
                </span>
            </div>
        </div>
    `;

    // 如果贝塞尔且有手柄，显示手柄坐标
    if (cont === 'C0' || cont === 'C1') {
        const inStr = anchor.handleIn ? `(${anchor.handleIn[0].toFixed(1)}, ${anchor.handleIn[1].toFixed(1)})` : '—';
        const outStr = anchor.handleOut ? `(${anchor.handleOut[0].toFixed(1)}, ${anchor.handleOut[1].toFixed(1)})` : '—';
        html += `
            <div class="dw-prop-group">
                <div class="dw-prop-title">🎛️ 手柄控制</div>
                <div class="dw-param-row">
                    <label>入段手柄 (P2)</label>
                    <span class="dw-param-value" style="font-size:11px;">${inStr}</span>
                </div>
                <div class="dw-param-row">
                    <label>出段手柄 (P1)</label>
                    <span class="dw-param-value" style="font-size:11px;">${outStr}</span>
                </div>
            </div>
        `;
    }

    // ---- 太阳独立控件（固定显示） ----
    html += `
        <div class="dw-prop-group">
            <div class="dw-prop-title">☀️ 太阳驱动 (独立周期)</div>
            <div class="dw-sun-compass">
                <div class="dw-sun-circle">
                    <div class="dw-sun-dot"></div>
                    <div class="dw-sun-line"></div>
                    <div style="position:absolute; bottom:-4px; left:50%; transform:translateX(-50%); font-size:8px; color:#64748b;">N</div>
                </div>
                <div class="dw-sun-info">
                    <div>方位角: <strong>120°</strong></div>
                    <div>高度角: <strong>45°</strong></div>
                    <div style="font-size:10px; color:#64748b;">周期: 60s | 纬度: 40°</div>
                </div>
            </div>
            <div class="dw-param-row">
                <label>周期 (Period)</label>
                <input type="range" min="10" max="300" value="60" style="width:120px;">
                <span class="dw-param-value">60s</span>
            </div>
            <div class="dw-param-row">
                <label>纬度 (Latitude)</label>
                <input type="range" min="-90" max="90" value="40" style="width:120px;">
                <span class="dw-param-value">40°</span>
            </div>
        </div>
    `;

    // ---- Rain Mapper（仅当选中降雨轨道时显示） ----
    if (trackId === 'rain') {
        html += `
            <div class="dw-prop-group">
                <div class="dw-prop-title">🌧️ 降雨物理映射 (RainMapper)</div>
                <div class="dw-param-row">
                    <label>积水倾向</label>
                    <input type="range" min="0" max="2" step="0.1" value="0.8" style="width:120px;">
                    <span class="dw-param-value">0.8</span>
                </div>
                <div class="dw-param-row">
                    <label>排水速率</label>
                    <input type="range" min="0" max="1" step="0.05" value="0.3" style="width:120px;">
                    <span class="dw-param-value">0.30</span>
                </div>
                <div class="dw-param-row">
                    <label>湿润速度</label>
                    <input type="range" min="0" max="2" step="0.1" value="1.2" style="width:120px;">
                    <span class="dw-param-value">1.2</span>
                </div>
                <div class="dw-param-row">
                    <label>干燥速率</label>
                    <input type="range" min="0" max="1" step="0.05" value="0.15" style="width:120px;">
                    <span class="dw-param-value">0.15</span>
                </div>
                <div class="dw-mini-map">
                    <div style="font-size:10px; color:#64748b; margin-bottom:4px;">降雨 → 积水 响应曲线</div>
                    <div class="dw-map-row">
                        <span>雨 0%</span>
                        <div class="bar"><div class="fill" style="width:10%;"></div></div>
                        <span>积 10%</span>
                    </div>
                    <div class="dw-map-row">
                        <span>雨 50%</span>
                        <div class="bar"><div class="fill" style="width:45%;"></div></div>
                        <span>积 45%</span>
                    </div>
                    <div class="dw-map-row">
                        <span>雨 100%</span>
                        <div class="bar"><div class="fill" style="width:85%;"></div></div>
                        <span>积 85%</span>
                    </div>
                </div>
            </div>
        `;
    }

    // ---- Fog Mapper（仅当选中雾轨道时显示） ----
    if (trackId === 'fog') {
        html += `
            <div class="dw-prop-group">
                <div class="dw-prop-title">🌫️ 雾映射 (FogMapper) · 预设: 浓雾</div>
                <div class="dw-param-row">
                    <label>密度曲线</label>
                    <span class="dw-param-value" style="font-size:11px;">线性 0→100</span>
                </div>
                <div class="dw-param-row">
                    <label>距离曲线</label>
                    <span class="dw-param-value" style="font-size:11px;">80→2 m</span>
                </div>
                <div class="dw-param-row">
                    <label>衰减曲线</label>
                    <span class="dw-param-value" style="font-size:11px;">0.5→3.0</span>
                </div>
                <div style="display:flex; gap:4px; margin-top:4px; flex-wrap:wrap;">
                    <span style="background:#2d3a4f; padding:2px 8px; border-radius:12px; font-size:10px; color:#94a3b8;">预设: 轻雾</span>
                    <span style="background:#2d3a4f; padding:2px 8px; border-radius:12px; font-size:10px; color:#94a3b8;">预设: 浓雾</span>
                    <span style="background:#3b82f6; padding:2px 8px; border-radius:12px; font-size:10px; color:white;">自定义</span>
                </div>
            </div>
        `;
    }

    panel.innerHTML = html;

    // 绑定连续性下拉事件
    const sel = document.getElementById('dwContinuitySelect');
    if (sel) {
        sel.addEventListener('change', function() {
            const newCont = this.value;
            const anchors = dwState.anchors[trackId];
            if (anchors && anchors[idx]) {
                anchors[idx].continuity = newCont;
                // 如果切换到 C2，清空手柄
                if (newCont === 'C2') {
                    anchors[idx].handleIn = null;
                    anchors[idx].handleOut = null;
                } else if (newCont === 'C1') {
                    // C1 需保证 handleIn == handleOut，简单处理
                    if (anchors[idx].handleIn) {
                        anchors[idx].handleOut = anchors[idx].handleIn;
                    } else {
                        // 生成默认手柄
                        const prev = idx > 0 ? anchors[idx-1] : null;
                        const next = idx < anchors.length-1 ? anchors[idx+1] : null;
                        const dt = next ? next.t - anchors[idx].t : 5;
                        anchors[idx].handleIn = [anchors[idx].t - dt/3, anchors[idx].val];
                        anchors[idx].handleOut = [anchors[idx].t + dt/3, anchors[idx].val];
                    }
                }
                renderCanvas();
                renderPropertyPanel();
                showNotification(`连续性切换为 ${newCont}`, 'info');
            }
        });
    }

    // 绑定滑块（占位）
    panel.querySelectorAll('input[type="range"]').forEach(slider => {
        slider.addEventListener('input', function() {
            const val = parseFloat(this.value);
            const display = this.closest('.dw-param-row')?.querySelector('.dw-param-value');
            if (display) display.textContent = val.toFixed(1);
        });
    });
}
