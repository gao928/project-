import { state, showNotification } from './core.js';
import { updatePreview } from './weather.js';

export function addDataset(seq) {
    let record = {
        id: "ds_" + Date.now(),
        name: `${seq.carlaMap} · ${seq.mode === "beginner" ? seq.preset : "专家"} · ${seq.durationSec}s/${seq.fps}fps · ${new Date().toLocaleTimeString()}`,
        timestamp: new Date().toLocaleString(),
        mode: seq.mode,
        framesCount: seq.totalFrames,
        duration: seq.durationSec,
        fps: seq.fps,
        activeSensors: seq.activeSensors,
        sensorsFullConfig: seq.sensorsFullConfig,
        map: seq.carlaMap,
        traffic: seq.traffic,
        savePath: seq.savePath,
        rawData: seq
    };
    state.datasets.unshift(record);
    renderDatasetList();
    state.lastSequence = seq;
    updatePreview();
}

export function renderDatasetList() {
    const container = document.getElementById("datasetList");
    if (!container) return;
    if (state.datasets.length === 0) {
        container.innerHTML = "<div style='text-align:center;color:#8da0bc;padding:32px 20px;'>✨ 生成后显示记录</div>";
        return;
    }
    let html = "";
    state.datasets.forEach(ds => {
        html += `<div class="dataset-item" data-id="${ds.id}">
                    <div class="dataset-name">${ds.name}</div>
                    <div class="dataset-meta">
                        <span>🗺️ ${ds.map}</span>
                        <span>🎛️ ${ds.mode === "beginner" ? "预设" : "专家"}</span>
                        <span>⏱️ ${ds.duration}s/${ds.fps}fps</span>
                        <span>🎞️ ${ds.framesCount}帧</span>
                        <span>🚗 ${ds.traffic?.vehiclesEnabled ? ds.traffic.vehicleCount + "辆" : "无车"}</span>
                        <span>📡 ${ds.activeSensors.length}个传感器启用</span>
                    </div>
                </div>`;
    });
    container.innerHTML = html;

    document.querySelectorAll('.dataset-item').forEach(item => {
        item.addEventListener('click', (e) => {
            e.stopPropagation();
            const id = item.getAttribute('data-id');
            const ds = state.datasets.find(d => d.id === id);
            if (ds) showNotification(`数据集: ${ds.name}`, 'info');
        });
    });
}

export function clearDatasets() {
    if (state.datasets.length > 0 && confirm('确定要清空所有数据集吗？')) {
        state.datasets = [];
        state.lastSequence = null;
        renderDatasetList();
        showNotification('数据集已清空', 'success');
    }
}

export function initDatasets() {
    renderDatasetList();
    const clearBtn = document.getElementById('clearDatasetsBtn');
    if (clearBtn) clearBtn.addEventListener('click', clearDatasets);
}