import { state, showNotification } from './core.js';
import { updatePreview } from './weather.js';

export function initTraffic() {
    const vehicleToggle = document.getElementById('vehicleToggle');
    const pedestrianToggle = document.getElementById('pedestrianToggle');
    const vehicleCountInput = document.getElementById('vehicleCount');
    const pedestrianCountInput = document.getElementById('pedestrianCount');

    // 设置初始状态
    if (vehicleToggle) {
        if (state.vehiclesEnabled) vehicleToggle.classList.add('active');
        vehicleToggle.addEventListener('click', () => {
            state.vehiclesEnabled = !state.vehiclesEnabled;
            vehicleToggle.classList.toggle('active', state.vehiclesEnabled);
            updatePreview();
            showNotification(`车辆${state.vehiclesEnabled ? '已启用' : '已禁用'}`, 'info');
        });
    }

    if (pedestrianToggle) {
        if (state.pedestrianEnabled) pedestrianToggle.classList.add('active');
        pedestrianToggle.addEventListener('click', () => {
            state.pedestrianEnabled = !state.pedestrianEnabled;
            pedestrianToggle.classList.toggle('active', state.pedestrianEnabled);
            updatePreview();
            showNotification(`行人${state.pedestrianEnabled ? '已启用' : '已禁用'}`, 'info');
        });
    }

    if (vehicleCountInput) {
        vehicleCountInput.value = state.vehicleCount;
        vehicleCountInput.addEventListener('input', (e) => {
            let v = parseInt(e.target.value);
            if (isNaN(v)) v = 0;
            state.vehicleCount = Math.min(50, Math.max(0, v));
            e.target.value = state.vehicleCount;
            updatePreview();
        });
    }

    if (pedestrianCountInput) {
        pedestrianCountInput.value = state.pedestrianCount;
        pedestrianCountInput.addEventListener('input', (e) => {
            let v = parseInt(e.target.value);
            if (isNaN(v)) v = 0;
            state.pedestrianCount = Math.min(50, Math.max(0, v));
            e.target.value = state.pedestrianCount;
            updatePreview();
        });
    }
}