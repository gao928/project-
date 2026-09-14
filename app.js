import { state, showNotification, initSavePathSelector, checkCarlaPath, selectCarlaPath, updateTotalFrames, updateCruiseRouteHint } from './core.js';
import { initSensors, resetAllSensors } from './sensors.js';
import { initSimulator, launchCarlaAir, stopCarlaAir } from './simulator.js';
import { initWeather, generateWeatherSequence, updatePreview } from './weather.js';
import { initTraffic } from './traffic.js';
import { initDatasets, addDataset } from './datasets.js';
import { initIssues } from './issues.js';
import { initRoutePlanning } from './route-planning.js';
import * as api from './api.js';

let lastManualOverrideId = 0;
let lastCruiseState = '';

function shortMapName(name) {
    return String(name || '').replace(/\\/g, '/').split('/').pop();
}

function startCruiseStatusMonitor() {
    setInterval(async () => {
        // 采集页提示随路线模式/地图变化实时刷新。
        updateCruiseRouteHint();
        try {
            const drone = await api.droneStatus();
            if (drone.control_state !== lastCruiseState) {
                lastCruiseState = drone.control_state;
                if (drone.control_state === 'taking_off') {
                    showNotification('无人机正在起飞至设定巡航高度，请稍候...', 'info');
                } else if (drone.control_state === 'planning_route') {
                    showNotification('正在规划道路巡航路线...', 'info');
                } else if (drone.control_state === 'auto_cruise') {
                    showNotification('自动巡航与数据采集已开始', 'success');
                } else if (drone.control_state === 'landed') {
                    showNotification('数据采集完成，无人机已自动降落', 'success');
                }
            }

            const override = drone.manual_override;
            if (override && override.id > lastManualOverrideId) {
                lastManualOverrideId = override.id;
                const message = `检测到飞行控制按键 ${override.key}，自动巡航已退出，控制权已交还给用户。`;
                showNotification(message, 'warning');
                window.alert(message);
            }
        } catch (error) {
            // 后端尚未启动时由既有健康检查负责提示。
        }
    }, 1000);
}

// ==================== 采集主流程 ====================
function validateParameters() {
    const totalFrames = state.collectDuration * state.sampleFps;
    if (totalFrames < 1 || totalFrames > 18000) {
        showNotification('总帧数应在1-18000之间', 'error');
        return false;
    }
    if (state.vehicleCount > 200) {
        showNotification(`车辆数量不能超过200`, 'error');
        return false;
    }
    if (state.pedestrianCount > 200) {
        showNotification(`行人数量不能超过200`, 'error');
        return false;
    }
    return true;
}

async function generateWithProgress() {
    if (!validateParameters()) return;

    let currentCarlaMap = null;
    try {
        const status = await api.getStatus();
        if (!status.connected) {
            showNotification('未连接到 CARLA-Air，请先点击「启动 Carla-Air」', 'warning');
            return;
        }
        currentCarlaMap = status.map;
    } catch (error) {
        showNotification('无法连接到后端服务，请重启应用', 'error');
        return;
    }

    const selectedMap = state.currentMap;
    // 后端返回的是完整 CARLA 地图路径，需统一成短名称后再比较，
    // 避免每次生成数据集都无谓地重载一次世界地图。
    if (shortMapName(currentCarlaMap) !== selectedMap) {
        showNotification(`正在切换地图到 ${selectedMap}...`, 'info');
        try {
            const switchResult = await api.loadMap(selectedMap);
            if (!switchResult.success) {
                showNotification(`地图切换失败: ${switchResult.error}`, 'error');
                return;
            }
            showNotification(`地图已切换到 ${selectedMap}`, 'success');
        } catch (error) {
            showNotification('地图切换请求失败，请检查后端', 'error');
            return;
        }
    }

    const sequence = generateWeatherSequence();
    console.log('请求帧数:', sequence.totalFrames);
    showNotification(`将生成 ${sequence.totalFrames} 帧`, 'info');

    try {
        const spawnResult = await api.spawnSensors(sequence.sensorsFullConfig, state.currentSavePath);
        if (!spawnResult.success) {
            showNotification('传感器孵化失败: ' + spawnResult.error, 'error');
            return;
        }
    } catch (error) {
        showNotification('传感器孵化请求失败', 'error');
        return;
    }

    const progressBar = document.createElement('div');
    progressBar.className = 'progress-bar';
    progressBar.innerHTML = '<div class="progress-fill"></div><span>0%</span>';
    document.body.appendChild(progressBar);

    const totalFrames = sequence.frames.length;
    const datasetId = 'ds_' + Date.now();

    try {
        showNotification('正在接管无人机并起飞，达到巡航高度后开始采集...', 'info');
        // 若在“地图与路线”页选择了“自定义路线”模式且保存的航线与当前地图匹配，
        // 则采集时沿该航线飞行；否则沿用当前地图的固定巡航航线。
        const routeId = (state.routeMode === 'custom' && state.customRouteMap === selectedMap)
            ? state.customRouteId
            : null;
        const result = await api.postWeatherSequence(
            sequence.frames,
            state.sampleFps,
            sequence.sensorsFullConfig,
            state.cruiseSpeed,
            state.cruiseAltitude,
            routeId
        );
        if (result.success) {
            for (let i = 0; i <= totalFrames; i += Math.ceil(totalFrames / 20)) {
                await new Promise(resolve => setTimeout(resolve, 50));
                const percent = Math.min(100, Math.floor((i / totalFrames) * 100));
                const fill = progressBar.querySelector('.progress-fill');
                const span = progressBar.querySelector('span');
                if (fill) fill.style.width = `${percent}%`;
                if (span) span.textContent = `${percent}%`;
            }
            if (window.api && window.api.saveDataset) {
                await window.api.saveDataset({
                    savePath: state.currentSavePath,
                    sequence: sequence,
                    datasetId: datasetId
                });
            }
            addDataset(sequence);
            showNotification(`数据集生成成功！共${totalFrames}帧数据`, 'success');
        } else {
            showNotification(`生成失败: ${result.error}`, 'error');
        }
    } catch (error) {
        console.error('API 调用失败:', error);
        showNotification(`生成失败: ${error.message || '请检查 Carla-Air 是否运行'}`, 'error');
    } finally {
        try {
            await api.stopSensors();
        } catch (e) {
            console.error('停止传感器失败:', e);
        }
    }
    setTimeout(() => progressBar.remove(), 500);
}

function initPageNavigation() {
    const navItems = document.querySelectorAll('.nav-item');
    function switchPage(pageId) {
        document.querySelectorAll('.page-container').forEach(page => page.classList.remove('active-page'));
        const targetPage = document.getElementById(pageId);
        if (targetPage) targetPage.classList.add('active-page');
        navItems.forEach(item => {
            const targetPageId = item.getAttribute('data-page');
            if (targetPageId === pageId) item.classList.add('active');
            else item.classList.remove('active');
        });
    }
    navItems.forEach(item => {
        item.addEventListener('click', () => {
            const pageId = item.getAttribute('data-page');
            if (pageId) switchPage(pageId);
        });
    });
    const activeNav = document.querySelector('.nav-item.active');
    if (activeNav) switchPage(activeNav.getAttribute('data-page'));
}

function bindGlobalEvents() {
    const durationInput = document.getElementById('collectDuration');
    const fpsInput = document.getElementById('sampleFps');
    const durationMinus = document.getElementById('durationMinusBtn');
    const durationPlus = document.getElementById('durationPlusBtn');
    const fpsMinus = document.getElementById('fpsMinusBtn');
    const fpsPlus = document.getElementById('fpsPlusBtn');

    const updateHandler = () => {
        updateTotalFrames();
        updatePreview();
    };

    if (durationInput) durationInput.addEventListener('input', updateHandler);
    if (fpsInput) fpsInput.addEventListener('input', updateHandler);
    if (durationMinus) durationMinus.addEventListener('click', () => {
        let val = parseInt(durationInput.value) || 30;
        val = Math.max(1, val - 5);
        durationInput.value = val;
        updateHandler();
    });
    if (durationPlus) durationPlus.addEventListener('click', () => {
        let val = parseInt(durationInput.value) || 30;
        val = Math.min(300, val + 5);
        durationInput.value = val;
        updateHandler();
    });
    if (fpsMinus) fpsMinus.addEventListener('click', () => {
        let val = parseInt(fpsInput.value) || 10;
        val = Math.max(1, val - 5);
        fpsInput.value = val;
        updateHandler();
    });
    if (fpsPlus) fpsPlus.addEventListener('click', () => {
        let val = parseInt(fpsInput.value) || 10;
        val = Math.min(60, val + 5);
        fpsInput.value = val;
        updateHandler();
    });

    const generateBtn = document.getElementById('generateBtn');
    if (generateBtn) generateBtn.addEventListener('click', generateWithProgress);

    const resetSensorsBtn = document.getElementById('resetSensorsBtn');
    if (resetSensorsBtn) resetSensorsBtn.addEventListener('click', () => {
        resetAllSensors();
    });

    // 巡航高度和速度输入绑定
    const altitudeInput = document.getElementById('cruiseAltitudeInput');
    const speedInput = document.getElementById('cruiseSpeedInput');

    if (altitudeInput) {
        altitudeInput.addEventListener('input', (e) => {
            const val = parseFloat(e.target.value);
            if (!isNaN(val) && val >= 10 && val <= 200) {
                state.cruiseAltitude = val;
            }
        });
        state.cruiseAltitude = parseFloat(altitudeInput.value) || 45;
    }

    if (speedInput) {
        speedInput.addEventListener('input', (e) => {
            const val = parseFloat(e.target.value);
            if (!isNaN(val) && val >= 1 && val <= 30) {
                state.cruiseSpeed = val;
            }
        });
        state.cruiseSpeed = parseFloat(speedInput.value) || 8;
    }
}

function initKeyboardShortcuts() {
    document.addEventListener('keydown', (e) => {
        if (e.ctrlKey && e.key === 'g') {
            e.preventDefault();
            generateWithProgress();
        }
        if (e.ctrlKey && e.key === 'l') {
            e.preventDefault();
            launchCarlaAir();
        }
    });
}

async function init() {
    initSavePathSelector();

    initSimulator();
    initSensors();
    initWeather();
    initTraffic();
    initDatasets();
    initIssues();
    initRoutePlanning();

    initPageNavigation();
    bindGlobalEvents();
    initKeyboardShortcuts();
    startCruiseStatusMonitor();

    updatePreview();
    updateCruiseRouteHint();

    if (window.api && window.api.on) {
        window.api.on('need-carla-path', () => {
            showNotification('需要配置 Carla-Air 路径才能使用', 'warning');
            selectCarlaPath();
        });
        window.api.on('need-manual-config', () => {
            showNotification('请先填写并保存手动配置（Carla-Air 路径和 Python 解释器路径）', 'warning');
            const simulatorNav = document.querySelector('.nav-item[data-page="page-simulator"]');
            if (simulatorNav) simulatorNav.click();
        });
    }

    showNotification('平台已就绪，请点击「生成数据集」按钮开始采集', 'info');
}

init();
