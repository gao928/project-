const { app, BrowserWindow, ipcMain, dialog, Menu } = require('electron');
const { spawn, exec } = require('child_process');
const path = require('path');
const fs = require('fs');

let pythonProcess = null;
let mainWindow = null;
const configPath = path.join(app.getPath('userData'), 'config.json');
const BACKEND_VERSION = '7.7-map-alignment-v2';

function requestBackendVersion() {
    return new Promise((resolve) => {
        const http = require('http');
        // 用 127.0.0.1 而不是 localhost：Windows 上 localhost 可能先解析到 ::1，
        // 而后端只监听 IPv4，会导致"后端明明在跑却判定连不上"，进而重复拉起第二个后端。
        const req = http.request('http://127.0.0.1:5000/api/version', { timeout: 3000 }, (res) => {
            let data = '';
            res.on('data', chunk => data += chunk);
            res.on('end', () => {
                try {
                    const body = JSON.parse(data);
                    resolve({ reachable: true, version: body.version || null });
                } catch (error) {
                    resolve({ reachable: true, version: null });
                }
            });
        });
        req.on('error', () => resolve({ reachable: false, version: null }));
        req.on('timeout', () => req.destroy());
        req.end();
    });
}

function isProjectBackend() {
    return new Promise((resolve) => {
        const http = require('http');
        const req = http.request('http://127.0.0.1:5000/api/map/fixed-route?map=Town03', { timeout: 1500 }, (res) => {
            let data = '';
            res.on('data', chunk => data += chunk);
            res.on('end', () => {
                try {
                    const body = JSON.parse(data);
                    resolve(
                        res.statusCode === 200
                        && typeof body.alignment === 'string'
                        && Number.isFinite(Number(body.waypoint_count))
                    );
                } catch (error) {
                    resolve(false);
                }
            });
        });
        req.on('error', () => resolve(false));
        req.on('timeout', () => req.destroy());
        req.end();
    });
}

function stopStaleBackend() {
    if (process.platform !== 'win32') return Promise.resolve();
    return new Promise((resolve) => {
        exec('netstat -ano -p tcp | findstr LISTENING | findstr :5000', (error, stdout) => {
            if (error || !stdout) {
                resolve();
                return;
            }
            const pids = [...stdout.matchAll(/\s(\d+)\s*$/gm)].map(match => match[1]);
            if (!pids.length) {
                resolve();
                return;
            }
            let pending = pids.length;
            pids.forEach((pid) => {
                // 调用前已经由 isProjectBackend 确认为本项目旧 API，避免误杀其他 5000 端口服务。
                exec(`taskkill /PID ${pid} /F /T`, { windowsHide: true }, () => {
                    pending -= 1;
                    if (pending === 0) setTimeout(resolve, 300);
                });
            });
        });
    });
}

function loadConfig() {
    try {
        if (fs.existsSync(configPath)) {
            const data = fs.readFileSync(configPath, 'utf-8');
            return JSON.parse(data);
        }
    } catch (e) {
        console.error('读取配置失败:', e);
    }
    return {};
}

function saveConfig(config) {
    try {
        fs.writeFileSync(configPath, JSON.stringify(config, null, 2), 'utf-8');
        console.log('配置已保存');
    } catch (e) {
        console.error('保存配置失败:', e);
    }
}

function getPlatformDir() {
    if (app.isPackaged) {
        return path.dirname(app.getPath('exe'));
    } else {
        return __dirname;
    }
}

function getBackendPath() {
    if (app.isPackaged) {
        const backendPath = path.join(process.resourcesPath, 'backend.py');
        if (fs.existsSync(backendPath)) return backendPath;
        const platformDir = getPlatformDir();
        const exeBackend = path.join(platformDir, 'backend.py');
        if (fs.existsSync(exeBackend)) return exeBackend;
        return backendPath;
    } else {
        return path.join(__dirname, 'backend.py');
    }
}

async function startPythonBackend() {
    const config = loadConfig();
    const carlaAirPath = config.carlaAirPath;
    const pythonPath = config.pythonPath;
    if (!carlaAirPath || !pythonPath) {
        console.log('手动配置未完成，等待用户配置');
        if (mainWindow) {
            mainWindow.webContents.send('need-manual-config', {});
        }
        return false;
    }
    if (!fs.existsSync(carlaAirPath)) {
        console.error(`Carla-Air 路径无效: ${carlaAirPath}`);
        return false;
    }
    if (!fs.existsSync(pythonPath)) {
        console.error(`Python 解释器路径无效: ${pythonPath}`);
        return false;
    }
    const backendPath = getBackendPath();
    if (!fs.existsSync(backendPath)) {
        console.error('backend.py 不存在:', backendPath);
        return false;
    }
    console.log(`使用 Python: ${pythonPath}`);
    console.log(`Backend: ${backendPath}`);
    // 旧 Electron 异常退出后，旧后台可能仍占用 5000 端口；先做版本握手。
    const existingBackend = await requestBackendVersion();
    if (existingBackend.reachable && existingBackend.version === BACKEND_VERSION) {
        console.log('检测到当前版本后台已运行，直接复用');
        return true;
    }
    if (existingBackend.reachable) {
        if (await isProjectBackend()) {
            console.warn('检测到旧版本后台，正在清理 5000 端口');
            await stopStaleBackend();
        } else {
            console.error('5000 端口被非本项目服务占用，无法启动后台');
            return false;
        }
    }
    pythonProcess = spawn(pythonPath, [backendPath], {
        stdio: ['ignore', 'pipe', 'pipe'],
        detached: true,
        windowsHide: true
    });
    if (pythonProcess.stdout) {
        pythonProcess.stdout.on('data', (data) => {
            console.log(`[backend] ${data.toString().trim()}`);
        });
    }
    if (pythonProcess.stderr) {
        pythonProcess.stderr.on('data', (data) => {
            console.error(`[backend error] ${data.toString().trim()}`);
        });
    }
    pythonProcess.unref();
    return new Promise((resolve) => {
        const startTime = Date.now();
        const maxWait = 15000;
        const checkInterval = setInterval(async () => {
            try {
                const http = require('http');
                const req = http.request('http://localhost:5000/api/version', { timeout: 1000 }, (res) => {
                    if (res.statusCode === 200) {
                        clearInterval(checkInterval);
                        console.log('后端启动成功');
                        resolve(true);
                    }
                });
                req.on('error', () => {});
                req.end();
            } catch (e) {}
            if (Date.now() - startTime > maxWait) {
                clearInterval(checkInterval);
                console.error('后端启动超时');
                resolve(false);
            }
        }, 500);
    });
}

function stopPythonBackend() {
    if (pythonProcess) {
        if (process.platform === 'win32') {
            exec(`taskkill /pid ${pythonProcess.pid} /f /t`, { windowsHide: true });
        } else {
            pythonProcess.kill();
        }
        pythonProcess = null;
    }
}

function launchCarla() {
    return new Promise((resolve) => {
        const config = loadConfig();
        const carlaDir = config.carlaAirPath;
        if (!carlaDir) {
            resolve({ success: false, error: '未配置 Carla-Air 路径' });
            return;
        }
        const startScript = path.join(carlaDir, 'StartCarlaAir.bat');
        if (!fs.existsSync(startScript)) {
            resolve({ success: false, error: `启动脚本不存在: ${startScript}` });
            return;
        }
        const proc = spawn('cmd.exe', ['/c', 'start', '/b', startScript], {
            detached: true,
            stdio: 'ignore',
            windowsHide: true
        });
        proc.unref();
        resolve({ success: true, message: 'Carla-Air 启动命令已发送' });
    });
}

function stopCarla() {
    return new Promise((resolve) => {
        const config = loadConfig();
        const carlaDir = config.carlaAirPath;
        if (carlaDir) {
            const stopScript = path.join(carlaDir, 'StopCarlaAir.bat');
            if (fs.existsSync(stopScript)) {
                const proc = spawn('cmd.exe', ['/c', 'start', '/b', stopScript], {
                    detached: true,
                    stdio: 'ignore',
                    windowsHide: true
                });
                proc.unref();
            } else {
                exec('taskkill /f /im CarlaUE4.exe', { windowsHide: true });
            }
        } else {
            exec('taskkill /f /im CarlaUE4.exe', { windowsHide: true });
        }
        resolve({ success: true });
    });
}

async function selectDirectory() {
    const result = await dialog.showOpenDialog({
        properties: ['openDirectory', 'createDirectory'],
        title: '选择数据集保存位置'
    });
    return result.canceled ? null : result.filePaths[0];
}

async function selectFile() {
    const result = await dialog.showOpenDialog({
        properties: ['openFile'],
        title: '选择 Python 解释器',
        filters: [{ name: 'Python', extensions: ['exe'] }]
    });
    return result.canceled ? null : result.filePaths[0];
}

// ========== 关键：新增 JSON 文件选择器 ==========
async function selectJsonFile() {
    const result = await dialog.showOpenDialog({
        properties: ['openFile'],
        title: '选择 AirSim settings.json',
        filters: [{ name: 'JSON', extensions: ['json'] }]
    });
    return result.canceled ? null : result.filePaths[0];
}

async function saveDataset(data) {
    const { savePath, sequence, datasetId } = data;
    if (!savePath) {
        return { success: false, error: '未选择保存路径' };
    }
    try {
        if (!fs.existsSync(savePath)) {
            fs.mkdirSync(savePath, { recursive: true });
        }
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
        const jsonFileName = `dataset_${datasetId}_${timestamp}.json`;
        const csvFileName = `dataset_${datasetId}_${timestamp}.csv`;
        const jsonPath = path.join(savePath, jsonFileName);
        const exportData = {
            datasetId: datasetId,
            generatedAt: new Date().toISOString(),
            config: {
                mode: sequence.mode,
                preset: sequence.preset,
                curveType: sequence.curveType,
                durationSec: sequence.durationSec,
                fps: sequence.fps,
                totalFrames: sequence.totalFrames,
                carlaMap: sequence.carlaMap,
                activeSensors: sequence.activeSensors,
                traffic: sequence.traffic
            },
            frames: sequence.frames
        };
        fs.writeFileSync(jsonPath, JSON.stringify(exportData, null, 2), 'utf-8');
        const csvPath = path.join(savePath, csvFileName);
        let keys = new Set();
        sequence.frames.forEach(f => Object.keys(f.weatherParams).forEach(k => keys.add(k)));
        let headers = ['time_s', 'intensity_factor', ...Array.from(keys).sort()];
        let rows = sequence.frames.map(f => [
            f.timestamp,
            f.intensityFactor,
            ...Array.from(keys).map(k => f.weatherParams[k] ?? '')
        ]);
        let csv = [headers.join(','), ...rows.map(r => r.map(v => `"${v}"`).join(','))].join('\n');
        fs.writeFileSync(csvPath, '\uFEFF' + csv, 'utf-8');
        return { success: true, jsonPath, csvPath, message: `已保存到: ${savePath}` };
    } catch (error) {
        return { success: false, error: error.message };
    }
}

function setupIpcHandlers() {
    ipcMain.handle('carla:launch', async () => launchCarla());
    ipcMain.handle('carla:stop', async () => stopCarla());
    ipcMain.handle('dialog:selectDirectory', async () => selectDirectory());
    ipcMain.handle('dialog:selectFile', async () => selectFile());
    ipcMain.handle('dialog:selectJsonFile', async () => selectJsonFile()); // 注册 handler
    ipcMain.handle('dataset:save', async (event, data) => saveDataset(data));
    ipcMain.handle('app:getVersion', () => app.getVersion());
    ipcMain.handle('backend:status', async () => {
        return new Promise((resolve) => {
            const http = require('http');
            const req = http.request('http://localhost:5000/api/status', { timeout: 1000 }, (res) => {
                let data = '';
                res.on('data', chunk => data += chunk);
                res.on('end', () => {
                    try {
                        resolve(JSON.parse(data));
                    } catch (e) {
                        resolve({ connected: false });
                    }
                });
            });
            req.on('error', () => resolve({ connected: false }));
            req.end();
        });
    });
    ipcMain.handle('save-manual-config', (event, config) => {
        const existing = loadConfig();
        existing.carlaAirPath = config.carlaPath;
        existing.pythonPath = config.pythonPath;
        saveConfig(existing);
        return { success: true };
    });
    ipcMain.handle('get-carla-path', () => {
        const config = loadConfig();
        const carlaPath = config.carlaAirPath;
        return { hasPath: !!carlaPath, path: carlaPath };
    });
    ipcMain.handle('set-carla-path', (event, carlaPath) => {
        if (carlaPath && fs.existsSync(path.join(carlaPath, 'StartCarlaAir.bat'))) {
            const config = loadConfig();
            config.carlaAirPath = carlaPath;
            saveConfig(config);
            return { success: true };
        }
        return { success: false, error: '无效的 Carla-Air 路径' };
    });
    ipcMain.handle('save-airsim-config', (event, { settingsPath }) => {
        const config = loadConfig();
        config.airsimSettingsPath = settingsPath;
        saveConfig(config);
        return { success: true };
    });
    ipcMain.handle('get-airsim-config', () => {
        const config = loadConfig();
        return { settingsPath: config.airsimSettingsPath || null };
    });
}

function createWindow() {
    mainWindow = new BrowserWindow({
        width: 1400,
        height: 800,
        minWidth: 1200,
        minHeight: 600,
        webPreferences: {
            nodeIntegration: false,
            contextIsolation: true,
            preload: path.join(__dirname, 'preload.js')
        },
        title: 'Drone Dataset Platform'
    });
    Menu.setApplicationMenu(null);
    mainWindow.maximize();
    mainWindow.loadFile('index.html');
    mainWindow.on('closed', () => {
        stopPythonBackend();
        mainWindow = null;
    });
}

app.whenReady().then(async () => {
    console.log('应用启动中...');
    createWindow();
    setupIpcHandlers();
    const backendStarted = await startPythonBackend();
    if (backendStarted) {
        console.log('后端自动启动成功');
    } else {
        console.log('后端启动失败，等待用户手动配置');
    }
});

app.on('window-all-closed', () => {
    stopPythonBackend();
    if (process.platform !== 'darwin') {
        app.quit();
    }
});

app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
        createWindow();
    }
});
