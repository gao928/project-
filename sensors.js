import { state, showNotification } from './core.js';

// ==================== 传感器定义 ====================
// Carla 传感器（15个）
const CARLA_SENSOR_LIST = [
    {
        id: "camera_rgb",
        nameCn: "RGB相机",
        source: "carla",
        defaultEnabled: true,
        position: { x: 0, y: 0, z: -1 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { width: 800, height: 600, fov: 90 }
    },
    {
        id: "camera_depth",
        nameCn: "深度相机",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { width: 800, height: 600, fov: 90 }
    },
    {
        id: "camera_semantic",
        nameCn: "语义分割相机",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { width: 800, height: 600, fov: 90 }
    },
    {
        id: "camera_instance",
        nameCn: "实例分割相机",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { width: 800, height: 600, fov: 90 }
    },
    {
        id: "camera_optical_flow",
        nameCn: "光流相机",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { width: 800, height: 600, fov: 90 }
    },
    {
        id: "camera_dvs",
        nameCn: "动态视觉传感器",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { width: 800, height: 600, fov: 90 }
    },
    {
        id: "lidar",
        nameCn: "激光雷达",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: -0.3 },
        rotation: { yaw: 0, pitch: 0, roll: 0 },
        params: { channels: 32, range: 100, points_per_second: 56000, rotation_frequency: 10 }
    },
    {
        id: "semantic_lidar",
        nameCn: "语义激光雷达",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: -0.3 },
        rotation: { yaw: 0, pitch: 0, roll: 0 },
        params: { channels: 32, range: 100, points_per_second: 56000, rotation_frequency: 10 }
    },
    {
        id: "radar",
        nameCn: "毫米波雷达",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: -0.2 },
        rotation: { yaw: 0, pitch: 0, roll: 0 },
        params: {}
    },
    {
        id: "gnss",
        nameCn: "全球定位系统",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.2 },
        rotation: { yaw: 0, pitch: 0, roll: 0 },
        params: {}
    },
    {
        id: "imu",
        nameCn: "惯性测量单元",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0 },
        rotation: { yaw: 0, pitch: 0, roll: 0 },
        params: {}
    },
    {
        id: "camera_normals",
        nameCn: "法线相机",
        source: "carla",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { width: 800, height: 600, fov: 90 }
    }
];

// AirSim 传感器（15个，保留供未来扩展）
const AIRSIM_SENSOR_LIST = [
    {
        id: "imu_airsim",
        nameCn: "惯性测量单元",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0 },
        rotation: { yaw: 0, pitch: 0, roll: 0 },
        params: {}
    },
    {
        id: "gps_airsim",
        nameCn: "全球定位系统",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.2 },
        rotation: { yaw: 0, pitch: 0, roll: 0 },
        params: {}
    },
    {
        id: "barometer_airsim",
        nameCn: "气压计",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0 },
        rotation: { yaw: 0, pitch: 0, roll: 0 },
        params: {}
    },
    {
        id: "magnetometer_airsim",
        nameCn: "磁力计",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0 },
        rotation: { yaw: 0, pitch: 0, roll: 0 },
        params: {}
    },
    {
        id: "airsim_scene",
        nameCn: "场景相机",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { image_type: 0, width: 800, height: 600, fov: 90 }
    },
    {
        id: "airsim_depth_planar",
        nameCn: "深度平面相机",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { image_type: 1, width: 800, height: 600, fov: 90 }
    },
    {
        id: "airsim_depth_perspective",
        nameCn: "深度透视相机",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { image_type: 2, width: 800, height: 600, fov: 90 }
    },
    {
        id: "airsim_depth_vis",
        nameCn: "深度可视化相机",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { image_type: 3, width: 800, height: 600, fov: 90 }
    },
    {
        id: "airsim_disparity_normalized",
        nameCn: "视差归一化相机",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { image_type: 4, width: 800, height: 600, fov: 90 }
    },
    {
        id: "airsim_segmentation",
        nameCn: "语义分割相机",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { image_type: 5, width: 800, height: 600, fov: 90 }
    },
    {
        id: "airsim_surface_normals",
        nameCn: "表面法线相机",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { image_type: 6, width: 800, height: 600, fov: 90 }
    },
    {
        id: "airsim_infrared",
        nameCn: "红外相机",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { image_type: 7, width: 800, height: 600, fov: 90 }
    },
    {
        id: "airsim_optical_flow",
        nameCn: "光流相机",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { image_type: 8, width: 800, height: 600, fov: 90 }
    },
    {
        id: "airsim_optical_flow_vis",
        nameCn: "光流可视化相机",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: 0.5 },
        rotation: { yaw: 0, pitch: -15, roll: 0 },
        params: { image_type: 9, width: 800, height: 600, fov: 90 }
    },
    {
        id: "lidar_airsim",
        nameCn: "激光雷达",
        source: "airsim",
        defaultEnabled: false,
        position: { x: 0, y: 0, z: -0.3 },
        rotation: { yaw: 0, pitch: 0, roll: 0 },
        params: { number_of_channels: 16, range: 100, points_per_second: 100000, rotation_frequency: 10 }
    }
];

const ALL_SENSORS = [...CARLA_SENSOR_LIST, ...AIRSIM_SENSOR_LIST];

// 参数定义
const CARLA_PARAM_DEFS = {
    camera_rgb: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    camera_depth: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    camera_semantic: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    camera_instance: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    camera_optical_flow: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    camera_dvs: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    lidar: [{ name: 'channels', label: '通道数', default: 32, step: 1 }, { name: 'range', label: '范围(米)', default: 100, step: 1 }, { name: 'points_per_second', label: '每秒点数', default: 56000, step: 1000 }, { name: 'rotation_frequency', label: '旋转频率(Hz)', default: 10, step: 1 }],
    semantic_lidar: [{ name: 'channels', label: '通道数', default: 32, step: 1 }, { name: 'range', label: '范围(米)', default: 100, step: 1 }, { name: 'points_per_second', label: '每秒点数', default: 56000, step: 1000 }, { name: 'rotation_frequency', label: '旋转频率(Hz)', default: 10, step: 1 }],
    radar: [],
    gnss: [],
    imu: [],
    collision: [],
    obstacle: [],
    camera_normals: [],
    camera_cosmos_vis: [],
    hss_lidar: []
};

const AIRSIM_PARAM_DEFS = {
    imu_airsim: [], gps_airsim: [], barometer_airsim: [], magnetometer_airsim: [],
    airsim_scene: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    airsim_depth_planar: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    airsim_depth_perspective: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    airsim_depth_vis: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    airsim_disparity_normalized: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    airsim_segmentation: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    airsim_surface_normals: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    airsim_infrared: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    airsim_optical_flow: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    airsim_optical_flow_vis: [{ name: 'width', label: '宽度', default: 800, step: 1 }, { name: 'height', label: '高度', default: 600, step: 1 }, { name: 'fov', label: '视野(度)', default: 90, step: 1 }],
    lidar_airsim: [{ name: 'number_of_channels', label: '通道数', default: 16, step: 1 }, { name: 'range', label: '范围(米)', default: 100, step: 1 }, { name: 'points_per_second', label: '每秒点数', default: 100000, step: 1000 }, { name: 'rotation_frequency', label: '旋转频率(Hz)', default: 10, step: 1 }]
};

const ALL_PARAM_DEFS = { ...CARLA_PARAM_DEFS, ...AIRSIM_PARAM_DEFS };

// ==================== 传感器配置管理 ====================
export function initSensorConfigs() {
    state.sensorFullConfig = {};
    state.sensorStates = {};
    for (const sensor of ALL_SENSORS) {
        state.sensorFullConfig[sensor.id] = {
            enabled: sensor.defaultEnabled,
            source: sensor.source,
            position: { ...sensor.position },
            rotation: { ...sensor.rotation },
            params: { ...sensor.params }
        };
        state.sensorStates[sensor.id] = sensor.defaultEnabled;
    }
}

export function renderSensorGrid() {
    const container = document.getElementById("sensorGrid");
    if (!container) return;
    container.innerHTML = "";

    for (const sensor of ALL_SENSORS) {
        const config = state.sensorFullConfig[sensor.id];
        if (!config) continue;

        const card = document.createElement("div");
        card.className = "sensor-config-card";

        const header = document.createElement("div");
        header.className = "sensor-config-header";
        header.innerHTML = `
            <div class="sensor-title">
                <span>${sensor.nameCn} (${sensor.id})</span>
                <span class="sensor-badge ${sensor.source}">${sensor.source}</span>
            </div>
            <div class="sensor-controls">
                <div class="switch ${config.enabled ? 'active' : ''}" data-id="${sensor.id}"></div>
                <button class="expand-btn">▼</button>
            </div>
        `;

        const content = document.createElement("div");
        content.className = "sensor-config-content";

        // 位置
        const posGroup = document.createElement("div");
        posGroup.className = "param-group";
        posGroup.innerHTML = `<div class="param-label">📍 位置(x向前，y向右，z向上) (米)</div><div class="param-inputs" id="pos-${sensor.id}"></div>`;
        const posDiv = posGroup.querySelector(".param-inputs");
        for (const axis of ["x", "y", "z"]) {
            const wrapper = document.createElement("div");
            wrapper.className = "input-wrapper";
            wrapper.innerHTML = `<span>${axis.toUpperCase()}</span><input type="number" step="0.01" value="${config.position[axis]}" data-axis="${axis}">`;
            posDiv.appendChild(wrapper);
        }

        // 旋转
        const rotGroup = document.createElement("div");
        rotGroup.className = "param-group";
        rotGroup.innerHTML = `<div class="param-label">🔄 旋转 (yaw：从上往下看绕z轴逆时针旋转；pitch：绕Y轴旋转，正值为抬头；roll：绕X轴旋转，正值为向右倾斜)(度)</div><div class="param-inputs" id="rot-${sensor.id}"></div>`;
        const rotDiv = rotGroup.querySelector(".param-inputs");
        for (const axis of ["yaw", "pitch", "roll"]) {
            const wrapper = document.createElement("div");
            wrapper.className = "input-wrapper";
            wrapper.innerHTML = `<span>${axis.toUpperCase()}</span><input type="number" step="1" value="${config.rotation[axis]}" data-rot="${axis}">`;
            rotDiv.appendChild(wrapper);
        }

        // 专属参数
        const paramDefs = ALL_PARAM_DEFS[sensor.id] || [];
        const paramsGroup = document.createElement("div");
        paramsGroup.className = "param-group";
        paramsGroup.innerHTML = `<div class="param-label">⚙️ 专属参数</div><div class="param-inputs" id="params-${sensor.id}"></div>`;
        const paramsDiv = paramsGroup.querySelector(".param-inputs");
        if (paramDefs.length === 0) {
            paramsDiv.innerHTML = '<div style="font-size:11px; color:#94a3b8;">无专属参数</div>';
        } else {
            paramDefs.forEach(pdef => {
                const val = config.params[pdef.name] !== undefined ? config.params[pdef.name] : pdef.default;
                const wrapper = document.createElement("div");
                wrapper.className = "input-wrapper";
                wrapper.innerHTML = `<span>${pdef.label}</span><input type="number" step="${pdef.step}" value="${val}" data-param="${pdef.name}">`;
                paramsDiv.appendChild(wrapper);
            });
        }

        content.appendChild(posGroup);
        content.appendChild(rotGroup);
        content.appendChild(paramsGroup);
        card.appendChild(header);
        card.appendChild(content);
        container.appendChild(card);

        // 事件绑定
        const switchEl = header.querySelector(".switch");
        switchEl.addEventListener("click", (e) => {
            e.stopPropagation();
            config.enabled = !config.enabled;
            switchEl.classList.toggle("active", config.enabled);
            state.sensorStates[sensor.id] = config.enabled;
            showNotification(`${sensor.nameCn} ${config.enabled ? "启用" : "禁用"}`, "info");
        });

        const expandBtn = header.querySelector(".expand-btn");
        expandBtn.addEventListener("click", () => {
            const isVisible = content.style.display !== "none";
            content.style.display = isVisible ? "none" : "block";
            expandBtn.textContent = isVisible ? "▼" : "▲";
        });

        posDiv.querySelectorAll("input[data-axis]").forEach(inp => {
            inp.addEventListener("change", () => {
                const axis = inp.dataset.axis;
                const val = parseFloat(inp.value);
                if (!isNaN(val)) config.position[axis] = val;
            });
        });
        rotDiv.querySelectorAll("input[data-rot]").forEach(inp => {
            inp.addEventListener("change", () => {
                const axis = inp.dataset.rot;
                const val = parseFloat(inp.value);
                if (!isNaN(val)) config.rotation[axis] = val;
            });
        });
        if (paramDefs.length) {
            paramsDiv.querySelectorAll("input[data-param]").forEach(inp => {
                inp.addEventListener("change", () => {
                    const pname = inp.dataset.param;
                    const val = parseFloat(inp.value);
                    if (!isNaN(val)) config.params[pname] = val;
                });
            });
        }
    }
}

export function resetAllSensors() {
    initSensorConfigs();
    renderSensorGrid();
    showNotification("所有传感器已重置为默认配置", "success");
}

export function initSensors() {
    initSensorConfigs();
    renderSensorGrid();
}