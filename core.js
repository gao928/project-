export const CONFIG = {
    SIMULATION: {
        DEFAULT_DURATION: 30,
        DEFAULT_FPS: 10,
        TIME_STEP: 0.05,
        MAX_VEHICLES: 200,
        MAX_PEDESTRIANS: 200
    },
    SENSORS: {
        DEFAULT_ENABLED: ['camera_rgb', 'lidar', 'gnss', 'imu']
    }
};

export const state = {
    sensorStates: {},
    sensorFullConfig: {},
    currentMode: "beginner",
    currentPresetId: null,
    expertParams: {},
    currentMap: "Town10HD",
    cruiseAltitude: 45,
    cruiseSpeed: 8,
    // 巡航路线模式：'fixed' 固定航线 | 'custom' 自定义路线（保存自定义路线后自动切换）
    routeMode: 'fixed',
    customRouteId: null,
    customRouteMap: null,
    carlaSimulatedRunning: false,
    backendReady: false,
    vehiclesEnabled: false,
    pedestrianEnabled: false,
    vehicleCount: 18,
    pedestrianCount: 12,
    collectDuration: 30,
    sampleFps: 10,
    currentSavePath: './datasets/',
    datasets: [],
    lastSequence: null,
    issues: [],
    carlaStatusInterval: null,
    backendCheckInterval: null,
    airsimSettingsPath: '',
};

// 通知队列
let notificationQueue = [];
const recentMessageMap = new Map();
let notificationIdCounter = 0;

function cleanupRecentMessage(message) {
    setTimeout(() => {
        recentMessageMap.delete(message);
    }, 5000);
}

function removeNotificationById(id) {
    const index = notificationQueue.findIndex(item => item.id === id);
    if (index === -1) return;
    const item = notificationQueue[index];
    if (item.timerId) {
        clearTimeout(item.timerId);
    }
    if (item.element) {
        item.element.style.animation = 'slideOut 0.3s ease';
        setTimeout(() => {
            if (item.element && item.element.parentNode) {
                item.element.remove();
            }
        }, 300);
    }
    notificationQueue.splice(index, 1);
    renderNotifications();
}

function renderNotifications() {
    const baseBottom = 20;
    const gap = 10;
    const visibleItems = notificationQueue.filter(item => item.element !== null);
    let accumulatedBottom = baseBottom;
    for (let i = visibleItems.length - 1; i >= 0; i--) {
        const item = visibleItems[i];
        const el = item.element;
        if (!el) continue;
        el.style.bottom = `${accumulatedBottom}px`;
        accumulatedBottom += 56 + gap;
    }
}

export function showNotification(message, type = 'info') {
    if (recentMessageMap.has(message)) {
        const existing = recentMessageMap.get(message);
        existing.count += 1;
        const queueItem = notificationQueue.find(item => item.message === message);
        if (queueItem && queueItem.element) {
            const textSpan = queueItem.element.querySelector('.notification-text');
            if (textSpan) {
                textSpan.textContent = `${message} (x${existing.count})`;
            }
        }
        return;
    }

    recentMessageMap.set(message, { count: 1 });
    cleanupRecentMessage(message);

    const isError = (type === 'error');
    const isWarning = (type === 'warning');
    const duration = isError ? Infinity : (isWarning ? 5000 : 3000);

    if (notificationQueue.length >= 3) {
        const hasHighPriority = notificationQueue.some(item => item.type === 'error' || item.type === 'warning');
        if (!isError && !isWarning && hasHighPriority) {
            console.debug(`[通知] 队列已满且存在高优先级消息，丢弃: ${message}`);
            return;
        }
        const oldest = notificationQueue[0];
        if (oldest.element && oldest.element.parentNode) {
            oldest.element.remove();
        }
        if (oldest.timerId) clearTimeout(oldest.timerId);
        notificationQueue.shift();
    }

    const toast = document.createElement('div');
    toast.className = `notification`;
    const bgColor = type === 'error' ? '#ef4444' :
                    type === 'success' ? '#10b981' :
                    type === 'warning' ? '#f59e0b' : '#3b82f6';
    toast.style.background = bgColor;
    toast.style.color = 'white';
    toast.style.display = 'flex';
    toast.style.alignItems = 'center';
    toast.style.gap = '8px';
    toast.style.padding = '12px 16px';
    toast.style.borderRadius = '12px';
    toast.style.boxShadow = '0 4px 12px rgba(0,0,0,0.15)';
    toast.style.zIndex = '1000';
    toast.style.position = 'fixed';
    toast.style.right = '20px';
    toast.style.fontSize = '14px';
    toast.style.fontWeight = '500';
    toast.style.minWidth = '200px';
    toast.style.maxWidth = '380px';
    toast.style.transition = 'all 0.3s ease';
    toast.style.animation = 'slideIn 0.3s ease';
    toast.style.bottom = '20px';

    const iconSpan = document.createElement('span');
    const icon = type === 'error' ? '❌' :
                 type === 'success' ? '✅' :
                 type === 'warning' ? '⚠️' : 'ℹ️';
    iconSpan.textContent = icon;
    toast.appendChild(iconSpan);

    const textSpan = document.createElement('span');
    textSpan.className = 'notification-text';
    textSpan.textContent = message;
    toast.appendChild(textSpan);

    if (isError) {
        const closeBtn = document.createElement('span');
        closeBtn.textContent = '×';
        closeBtn.style.marginLeft = 'auto';
        closeBtn.style.cursor = 'pointer';
        closeBtn.style.fontSize = '18px';
        closeBtn.style.fontWeight = 'bold';
        closeBtn.style.padding = '0 4px';
        closeBtn.style.lineHeight = '1';
        closeBtn.style.opacity = '0.7';
        closeBtn.style.transition = 'opacity 0.2s';
        closeBtn.addEventListener('mouseenter', () => { closeBtn.style.opacity = '1'; });
        closeBtn.addEventListener('mouseleave', () => { closeBtn.style.opacity = '0.7'; });
        closeBtn.addEventListener('click', () => {
            const item = notificationQueue.find(it => it.id === id);
            if (item) {
                removeNotificationById(item.id);
            }
        });
        toast.appendChild(closeBtn);
    }

    document.body.appendChild(toast);

    const id = ++notificationIdCounter;
    const notificationItem = {
        id,
        message,
        type,
        element: toast,
        timerId: null,
    };

    if (!isError) {
        notificationItem.timerId = setTimeout(() => {
            removeNotificationById(id);
        }, duration);
    }

    notificationQueue.push(notificationItem);
    renderNotifications();
}

export function initSavePathSelector() {
    const selectPathBtn = document.getElementById('selectSavePathBtn');
    const savePathDisplay = document.getElementById('savePathDisplay');
    if (selectPathBtn) {
        selectPathBtn.addEventListener('click', async () => {
            if (window.api && window.api.selectDirectory) {
                const path = await window.api.selectDirectory();
                if (path) {
                    state.currentSavePath = path;
                    if (savePathDisplay) savePathDisplay.innerHTML = `📂 ${path}`;
                    showNotification(`保存位置已设置为: ${path}`, 'success');
                }
            } else {
                const mockPath = prompt('请输入保存路径:', state.currentSavePath);
                if (mockPath && mockPath.trim()) {
                    state.currentSavePath = mockPath.trim();
                    if (savePathDisplay) savePathDisplay.innerHTML = `📂 ${state.currentSavePath}`;
                    showNotification(`保存位置已设置为: ${state.currentSavePath}（演示模式）`, 'info');
                }
            }
        });
    }
}

export async function checkCarlaPath() {
    if (!window.api || !window.api.getCarlaPath) return;
    const result = await window.api.getCarlaPath();
    if (!result.hasPath) {
        showNotification('请先选择 Carla-Air 文件夹', 'warning');
        setTimeout(() => selectCarlaPath(), 1000);
    }
}

export async function selectCarlaPath() {
    const result = await window.api.selectCarlaDir();
    if (result.success) {
        showNotification(`Carla-Air 路径已设置: ${result.path}`, 'success');
        setTimeout(() => location.reload(), 1000);
    } else if (result.error) {
        showNotification(`设置失败: ${result.error}，请重新选择`, 'error');
        setTimeout(() => selectCarlaPath(), 2000);
    }
}

export function updateTotalFrames() {
    const duration = parseInt(document.getElementById('collectDuration').value) || 0;
    const fps = parseInt(document.getElementById('sampleFps').value) || 0;
    const total = duration * fps;
    const totalFramesInput = document.getElementById('totalFrames');
    if (totalFramesInput) totalFramesInput.value = total;
    state.collectDuration = duration;
    state.sampleFps = fps;
    return total;
}

// 采集页巡航卡片提示：当前采集将沿哪条航线飞行。
export function updateCruiseRouteHint() {
    const el = document.getElementById('collectCruiseRouteHint');
    if (!el) return;
    if (state.routeMode === 'custom' && state.customRouteId && state.customRouteMap === state.currentMap) {
        el.textContent = `✈️ 将沿自定义路线 ${state.customRouteId}（${state.customRouteMap}）采集`;
        el.style.color = '#2563eb';
    } else {
        el.textContent = '🚁 将沿当前地图固定航线采集';
        el.style.color = '#64748b';
    }
}