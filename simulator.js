import { state, showNotification } from './core.js';
import * as api from './api.js';

export function updateCarlaStatus(message, color) {
    const statusDiv = document.getElementById('carlaStatus');
    if (statusDiv) {
        statusDiv.innerHTML = message;
        statusDiv.style.color = color === 'green' ? '#10b981' : color === 'blue' ? '#3b82f6' : color === 'red' ? '#ef4444' : color === 'orange' ? '#f59e0b' : '#64748b';
    }
}

async function checkBackendAndUpdateUI() {
    try {
        const status = await api.getStatus();
        if (status && status.connected === true) {
            if (!state.backendReady) {
                state.backendReady = true;
                updateCarlaStatus('✅ Carla-Air 运行中', 'green');
            }
            return true;
        } else {
            if (state.backendReady) {
                updateCarlaStatus('⚫ Carla-Air 未启动', 'gray');
                state.backendReady = false;
            } else {
                updateCarlaStatus('✅ 后端已就绪，点击启动 Carla-Air', 'blue');
                state.backendReady = true;
            }
            return true;
        }
    } catch (error) {
        state.backendReady = false;
        updateCarlaStatus('⏳ 等待后端连接...', 'orange');
        return false;
    }
}

function startBackendHealthCheck() {
    checkBackendAndUpdateUI();
    state.backendCheckInterval = setInterval(checkBackendAndUpdateUI, 2000);
}

// 加载手动配置到输入框
function loadManualConfig() {
    const carlaPath = localStorage.getItem('manual_carla_path');
    const pythonPath = localStorage.getItem('manual_python_path');
    if (carlaPath) {
        const input = document.getElementById('carlaPathInput');
        if (input) input.value = carlaPath;
    }
    if (pythonPath) {
        const input = document.getElementById('pythonPathInput');
        if (input) input.value = pythonPath;
    }
}

// 保存手动配置
async function saveManualConfig() {
    const carlaPath = document.getElementById('carlaPathInput').value.trim();
    const pythonPath = document.getElementById('pythonPathInput').value.trim();
    if (!carlaPath || !pythonPath) {
        showNotification('请填写完整路径', 'warning');
        return;
    }
    localStorage.setItem('manual_carla_path', carlaPath);
    localStorage.setItem('manual_python_path', pythonPath);
    await window.api.saveManualConfig({ carlaPath, pythonPath });
    showNotification('配置已保存，请重启应用生效', 'success');
}

function browseCarlaPath() {
    window.api.selectDirectory().then(path => {
        if (path) document.getElementById('carlaPathInput').value = path;
    });
}

function browsePythonPath() {
    window.api.selectFile().then(path => {
        if (path) document.getElementById('pythonPathInput').value = path;
    });
}

// ========== AirSim 配置 ==========
function loadAirSimConfig() {
    const settingsPath = localStorage.getItem('airsim_settings_path') || '';
    const input = document.getElementById('airsimSettingsInput');
    if (input) {
        input.value = settingsPath;
        state.airsimSettingsPath = settingsPath;
    } else {
        state.airsimSettingsPath = settingsPath;
    }
}

function saveAirSimConfig() {
    const input = document.getElementById('airsimSettingsInput');
    if (!input) return;
    const path = input.value.trim();
    localStorage.setItem('airsim_settings_path', path);
    state.airsimSettingsPath = path;
    if (window.api && window.api.saveAirSimConfig) {
        window.api.saveAirSimConfig({ settingsPath: path });
    }
}

/** 连接已经在运行的 Carla-Air（不重启仿真器） */
export async function connectExistingCarla() {
    updateCarlaStatus('🔗 正在连接已有 Carla-Air ...', 'blue');
    showNotification('正在连接已运行的 Carla-Air（不会重启仿真器）...', 'info');
    try {
        const result = await api.connectCarla();
        if (result.success) {
            state.carlaSimulatedRunning = true;
            state.backendReady = true;
            updateCarlaStatus(`✅ 已连接 ${result.map}`, 'green');
            showNotification(
                result.drone_found
                    ? `已连接到 ${result.map}，已找到无人机`
                    : `已连接到 ${result.map}，但未找到无人机 actor（请检查 AirSim settings.json 的 AutoCreate）`,
                result.drone_found ? 'success' : 'warning'
            );
        } else {
            updateCarlaStatus('⚫ 未连接', 'gray');
            showNotification('连接失败: ' + (result.error || '未知错误'), 'error');
        }
    } catch (error) {
        updateCarlaStatus('⚫ 未连接', 'gray');
        showNotification('连接失败: ' + error.message, 'error');
    }
}

export async function launchCarlaAir() {
    const carlaPath = localStorage.getItem('manual_carla_path');
    const pythonPath = localStorage.getItem('manual_python_path');
    if (!carlaPath || !pythonPath) {
        showNotification('请先在手动配置中填写并保存 Carla-Air 和 Python 路径', 'warning');
        return;
    }

    if (state.carlaSimulatedRunning) {
        updateCarlaStatus('⚠️ Carla-Air 已在运行中', 'orange');
        return;
    }

    const quality = document.querySelector('.quality-option.active')?.getAttribute('data-quality') || 'Epic';
    const map = state.currentMap;

    const airsimParams = {
        settingsPath: state.airsimSettingsPath,
        sensors: {}
    };

    let foundCamera = false;
    for (const [id, cfg] of Object.entries(state.sensorFullConfig)) {
        if (cfg.source === 'airsim') {
            airsimParams.sensors[id] = cfg.enabled;
            if (cfg.enabled && cfg.params && id.startsWith('airsim_') && cfg.params.width !== undefined && !foundCamera) {
                airsimParams.width = cfg.params.width;
                airsimParams.height = cfg.params.height;
                airsimParams.fov = cfg.params.fov || 90;
                foundCamera = true;
            }
            if (cfg.enabled && id === 'lidar_airsim' && cfg.params) {
                airsimParams.lidarChannels = cfg.params.number_of_channels || 16;
                airsimParams.lidarRange = cfg.params.range || 100;
                airsimParams.lidarPointsPerSecond = cfg.params.points_per_second || 100000;
                airsimParams.lidarRotationFrequency = cfg.params.rotation_frequency || 10;
            }
        }
    }
    if (!foundCamera) {
        airsimParams.width = 800;
        airsimParams.height = 600;
        airsimParams.fov = 90;
    }
    if (!airsimParams.lidarChannels) {
        airsimParams.lidarChannels = 16;
        airsimParams.lidarRange = 100;
        airsimParams.lidarPointsPerSecond = 100000;
        airsimParams.lidarRotationFrequency = 10;
    }

    updateCarlaStatus('🚀 正在启动 Carla-Air (含 AirSim 配置)...', 'blue');
    showNotification(`正在启动 Carla-Air (地图: ${map}, 画质: ${quality})，请稍候...`, 'info');

    try {
        const result = await api.launchCarla(quality, map, airsimParams);
        if (result.success) {
            state.carlaSimulatedRunning = true;
            updateCarlaStatus('✅ Carla-Air 运行中', 'green');
            showNotification('Carla-Air 启动成功', 'success');
        } else {
            updateCarlaStatus('❌ 启动失败: ' + (result.error || '未知错误'), 'red');
            showNotification('启动失败: ' + (result.error || '请检查 Carla-Air 路径'), 'error');
        }
    } catch (error) {
        console.error('启动 Carla-Air 失败:', error);
        updateCarlaStatus('❌ 无法连接后端', 'red');
        showNotification('无法连接后端服务，请重启应用', 'error');
    }
}

export async function stopCarlaAir() {
    if (!state.carlaSimulatedRunning) {
        updateCarlaStatus('⚠️ Carla-Air 未运行', 'orange');
        return;
    }
    updateCarlaStatus('⏹️ 正在停止...', 'orange');

    try {
        const result = await api.stopCarla();
        if (result.success) {
            state.carlaSimulatedRunning = false;
            updateCarlaStatus('⚫ 已停止', 'gray');
            showNotification('Carla-Air 已停止', 'info');
        } else {
            updateCarlaStatus('❌ 停止失败', 'red');
        }
    } catch (error) {
        console.error('停止 Carla-Air 失败:', error);
        updateCarlaStatus('❌ 无法连接后端', 'red');
    }
}

export function initSimulator() {
    loadManualConfig();
    loadAirSimConfig();

    const browseCarlaBtn = document.getElementById('browseCarlaPathBtn');
    const browsePythonBtn = document.getElementById('browsePythonBtn');
    const saveConfigBtn = document.getElementById('saveConfigBtn');
    if (browseCarlaBtn) browseCarlaBtn.addEventListener('click', browseCarlaPath);
    if (browsePythonBtn) browsePythonBtn.addEventListener('click', browsePythonPath);
    if (saveConfigBtn) saveConfigBtn.addEventListener('click', saveManualConfig);

    // ===== AirSim 浏览按钮（使用 selectJsonFile） =====
    const browseAirSimBtn = document.getElementById('browseAirSimSettingsBtn');
    if (browseAirSimBtn) {
        browseAirSimBtn.addEventListener('click', () => {
            console.log('[Simulator] 点击 AirSim 浏览按钮');
            if (window.api && window.api.selectJsonFile) {
                window.api.selectJsonFile().then(path => {
                    if (path) {
                        document.getElementById('airsimSettingsInput').value = path;
                        saveAirSimConfig();
                        showNotification('AirSim 路径已选择', 'success');
                    }
                }).catch(err => {
                    console.error('[Simulator] selectJsonFile 错误:', err);
                    showNotification('选择文件失败，请查看控制台', 'error');
                });
            } else {
                console.error('[Simulator] window.api.selectJsonFile 不可用');
                showNotification('当前环境不支持选择 JSON 文件，请重启应用', 'error');
            }
        });
    } else {
        console.warn('[Simulator] 未找到 #browseAirSimSettingsBtn');
    }

    const airsimInput = document.getElementById('airsimSettingsInput');
    if (airsimInput) {
        airsimInput.addEventListener('change', saveAirSimConfig);
    }

    // 画质选项高亮
    const qualityEpic = document.querySelector('.quality-option[data-quality="Epic"]');
    if (qualityEpic) {
        qualityEpic.classList.add('active');
        qualityEpic.style.background = '#3b82f6';
        qualityEpic.style.color = 'white';
    }
    document.querySelectorAll('.quality-option').forEach(opt => {
        opt.addEventListener('click', () => {
            document.querySelectorAll('.quality-option').forEach(o => {
                o.classList.remove('active');
                o.style.background = '#f1f5f9';
                o.style.color = '#1e293b';
            });
            opt.classList.add('active');
            opt.style.background = '#3b82f6';
            opt.style.color = 'white';
        });
    });

    const mapSelect = document.getElementById('carlaStartMap');
    if (mapSelect) {
        mapSelect.addEventListener('change', (e) => {
            state.currentMap = e.target.value;
        });
    }

    const launchBtn = document.getElementById('launchCarlaBtn');
    const stopBtn = document.getElementById('stopCarlaBtn');
    if (launchBtn) launchBtn.addEventListener('click', launchCarlaAir);
    if (stopBtn) stopBtn.addEventListener('click', stopCarlaAir);
    const connectBtn = document.getElementById('connectCarlaBtn');
    if (connectBtn) connectBtn.addEventListener('click', connectExistingCarla);

    startBackendHealthCheck();
}