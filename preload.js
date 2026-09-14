const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('api', {
    // Carla-Air 控制
    launchCarla: () => ipcRenderer.invoke('carla:launch'),
    stopCarla: () => ipcRenderer.invoke('carla:stop'),

    // 文件对话框
    selectDirectory: () => ipcRenderer.invoke('dialog:selectDirectory'),
    selectFile: () => ipcRenderer.invoke('dialog:selectFile'),
    selectJsonFile: () => ipcRenderer.invoke('dialog:selectJsonFile'),

    // 数据集保存
    saveDataset: (data) => ipcRenderer.invoke('dataset:save', data),

    // 应用信息
    getVersion: () => ipcRenderer.invoke('app:getVersion'),

    // 后端状态检查
    checkBackendStatus: () => ipcRenderer.invoke('backend:status'),

    // Carla-Air 路径配置
    selectCarlaDir: () => ipcRenderer.invoke('select-carla-dir'),
    getCarlaPath: () => ipcRenderer.invoke('get-carla-path'),
    setCarlaPath: (path) => ipcRenderer.invoke('set-carla-path', path),

    // 手动配置保存
    saveManualConfig: (config) => ipcRenderer.invoke('save-manual-config', config),

    // AirSim 配置
    saveAirSimConfig: (config) => ipcRenderer.invoke('save-airsim-config', config),
    getAirSimConfig: () => ipcRenderer.invoke('get-airsim-config'),

    // 监听事件
    on: (channel, callback) => {
        const validChannels = ['carla:statusChanged', 'dataset:saved', 'need-carla-path', 'need-manual-config'];
        if (validChannels.includes(channel)) {
            ipcRenderer.on(channel, (event, ...args) => callback(...args));
        }
    }
});