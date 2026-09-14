const API_BASE_URL = 'http://localhost:5000/api';

async function request(endpoint, options = {}) {
    const response = await fetch(`${API_BASE_URL}${endpoint}`, {
        headers: { 'Content-Type': 'application/json' },
        ...options
    });
    if (!response.ok) {
        let errorText = `HTTP ${response.status}`;
        try {
            const body = await response.json();
            errorText = body.error || errorText;
        } catch (error) {
            // 非 JSON 错误响应保留状态码，避免掩盖原始网络问题。
        }
        throw new Error(errorText);
    }
    return response.json();
}

export function getStatus() {
    return request('/status');
}

export function launchCarla(quality, map, airsimParams = null) {
    return request('/carla/launch', {
        method: 'POST',
        body: JSON.stringify({ quality, map, airsimParams })
    });
}

export function connectCarla() {
    return request('/carla/connect', { method: 'POST' });
}

export function stopCarla() {
    return request('/carla/stop', { method: 'POST' });
}

export function loadMap(map) {
    return request('/map/load', {
        method: 'POST',
        body: JSON.stringify({ map })
    });
}

export function getMapTopology(map = '', sampleDistance = 3) {
    const params = new URLSearchParams({
        sample_distance: String(sampleDistance)
    });
    if (map) params.set('map', map);
    return request(`/map/topology?${params.toString()}`);
}

export function getFixedRoute(map) {
    const params = new URLSearchParams({ map });
    return request(`/map/fixed-route?${params.toString()}`);
}

export function planRoute(points) {
    return request('/route/plan', {
        method: 'POST',
        body: JSON.stringify({ points })
    });
}

export function saveCustomRoute(points, altitude = 45) {
    return request('/route/save', {
        method: 'POST',
        body: JSON.stringify({ points, altitude })
    });
}

export function listCustomRoutes(map) {
    return request(`/route/custom?map=${encodeURIComponent(map)}`);
}

export function spawnSensors(sensorConfigs, saveDir) {
    return request('/sensors/spawn', {
        method: 'POST',
        body: JSON.stringify({ sensorConfigs, saveDir })
    });
}

export function stopSensors() {
    return request('/sensors/stop', { method: 'POST' });
}

export function postWeatherSequence(frames, fps, sensorsFullConfig, cruiseSpeed = 8, cruiseAltitude = 45, routeId = null) {
    const body = {
        frames,
        fps,
        sensorsFullConfig,
        cruiseSpeed,
        cruiseAltitude
    };
    if (routeId) body.route_id = routeId;
    return request('/weather/sequence', {
        method: 'POST',
        body: JSON.stringify(body)
    });
}

export function droneConnect(airsimPort = 41451, altitude = 45) {
    return request('/drone/connect', {
        method: 'POST',
        body: JSON.stringify({ airsim_port: airsimPort, altitude })
    });
}

export function droneStatus() {
    return request('/drone/status');
}

export function droneCruiseStart(speed = 8, altitude = 45, routeId = null) {
    const body = { speed, altitude };
    if (routeId) body.route_id = routeId;
    return request('/drone/cruise/start', {
        method: 'POST',
        body: JSON.stringify(body)
    });
}

export function droneCruiseStop() {
    return request('/drone/cruise/stop', { method: 'POST' });
}

export function droneLand() {
    return request('/drone/land', { method: 'POST' });
}
