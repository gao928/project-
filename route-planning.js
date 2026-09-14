import { getStatus, loadMap, getMapTopology, getFixedRoute, planRoute, saveCustomRoute, droneCruiseStart, droneCruiseStop, droneLand, droneStatus } from './api.js';
import { showNotification, state } from './core.js';

const SVG_NS = 'http://www.w3.org/2000/svg';

// 缩放 / 平移参数。
const ZOOM_STEP = 1.18;          // 滚轮每格的缩放比
const MAX_ZOOM = 40;             // 最多放大到自适应视图的 40 倍
const PAN_MARGIN_RATIO = 0.08;   // 平移时允许越出地图边界多少（相对地图尺寸）
const DRAG_THRESHOLD_PX = 4;     // 超过多少像素才算"拖动"（避免点选被判成拖动）

// 规划状态：供渲染与点击交互共享。
let viewTransform = null;      // {minX, maxY, padding, width, height}
let baseViewBox = null;        // 自适应 fit 视图，缩放/平移后用于重置
let selectedPoints = [];       // 用户点击的 CARLA 坐标 [x, y]
let plannedPolyline = null;    // 后端返回的 CARLA 折线 [[x,y,z], ...]
let savedRouteId = null;       // 最近一次成功保存的自定义航线 id
let savedRouteMap = null;      // 该自定义航线所属地图（短名称）
let droneCarlaPosition = null; // 无人机当前 CARLA 平面坐标（用于在地图上画出它）
let mapOffset = null;          // AirSim -> CARLA 的坐标偏移（来自固定航线接口的 alignment）
let pointsDirty = false;       // 图上画的点是否还没保存成自定义航线

function shortMapName(name) {
    return String(name || '').replace(/\\/g, '/').split('/').pop();
}

function createSvgElement(name, attributes = {}) {
    const element = document.createElementNS(SVG_NS, name);
    Object.entries(attributes).forEach(([key, value]) => element.setAttribute(key, String(value)));
    return element;
}

function createStarPoints(cx, cy, outerRadius, innerRadius) {
    const points = [];
    for (let index = 0; index < 10; index += 1) {
        const angle = -Math.PI / 2 + index * Math.PI / 5;
        const radius = index % 2 === 0 ? outerRadius : innerRadius;
        points.push(`${(cx + Math.cos(angle) * radius).toFixed(2)},${(cy + Math.sin(angle) * radius).toFixed(2)}`);
    }
    return points.join(' ');
}

function createRouteArrowMarker() {
    const marker = createSvgElement('marker', {
        id: 'route-arrow',
        markerWidth: 8,
        markerHeight: 8,
        refX: 6,
        refY: 3,
        orient: 'auto',
        markerUnits: 'strokeWidth',
        viewBox: '0 0 6 6',
    });
    marker.appendChild(createSvgElement('path', { d: 'M 0 0 L 6 3 L 0 6 z', fill: '#f59e0b' }));
    return marker;
}

// CARLA 坐标 -> SVG 用户(base)坐标（与 renderTopology 的 toSvgPoint 保持一致）。
// 注意 base 坐标只由地图边界决定，与缩放/平移无关，因此叠加层不需要跟着重算。
function carlaToSvg(point) {
    if (!viewTransform) return [0, 0];
    return [
        Number(point[0]) - viewTransform.minX + viewTransform.padding,
        viewTransform.maxY - Number(point[1]) + viewTransform.padding,
    ];
}

// 浏览器坐标 -> SVG 用户(base)坐标。
// 必须走 getScreenCTM：它能同时正确处理 viewBox 的缩放/平移和 preserveAspectRatio 的留白。
// 旧写法按容器宽高线性换算，一旦"内容宽高比 ≠ 容器宽高比"就会整体偏移（点击位置偏掉）。
function svgClientToBase(clientX, clientY, svg) {
    if (!svg || typeof svg.createSVGPoint !== 'function' || typeof svg.getScreenCTM !== 'function') {
        return null;
    }
    const ctm = svg.getScreenCTM();
    if (!ctm) return null;
    const point = svg.createSVGPoint();
    point.x = clientX;
    point.y = clientY;
    const transformed = point.matrixTransform(ctm.inverse());
    if (!Number.isFinite(transformed.x) || !Number.isFinite(transformed.y)) return null;
    return { x: transformed.x, y: transformed.y };
}

// SVG base 坐标 -> CARLA 坐标。
function baseToCarla(basePoint) {
    if (!viewTransform || !basePoint) return null;
    return [
        basePoint.x - viewTransform.padding + viewTransform.minX,
        viewTransform.maxY - (basePoint.y - viewTransform.padding),
    ];
}

// 浏览器坐标 -> CARLA 坐标（反算，需已加载地图）。
function svgClientToCarla(clientX, clientY, svg) {
    return baseToCarla(svgClientToBase(clientX, clientY, svg));
}

// ---- 视图（缩放 / 平移）工具 ----

function readViewBox(svg) {
    const vb = svg.viewBox && svg.viewBox.baseVal;
    if (!vb || vb.width <= 0 || vb.height <= 0) return null;
    return { x: vb.x, y: vb.y, w: vb.width, h: vb.height };
}

function writeViewBox(svg, box) {
    svg.setAttribute('viewBox', `${box.x} ${box.y} ${box.w} ${box.h}`);
}

// 把视图限制在自适应范围附近，避免用户缩放后"迷失"在地图外面。
function clampViewBox(box) {
    if (!baseViewBox) return box;
    const marginX = baseViewBox.w * PAN_MARGIN_RATIO;
    const marginY = baseViewBox.h * PAN_MARGIN_RATIO;
    const next = { x: box.x, y: box.y, w: box.w, h: box.h };
    if (box.w >= baseViewBox.w) {
        next.x = baseViewBox.x;
    } else {
        next.x = Math.min(Math.max(box.x, baseViewBox.x - marginX), baseViewBox.x + baseViewBox.w + marginX - box.w);
    }
    if (box.h >= baseViewBox.h) {
        next.y = baseViewBox.y;
    } else {
        next.y = Math.min(Math.max(box.y, baseViewBox.y - marginY), baseViewBox.y + baseViewBox.h + marginY - box.h);
    }
    return next;
}

// 以 anchor（base 坐标）为不动点缩放：factor > 1 缩小，factor < 1 放大。
function zoomView(svg, anchorX, anchorY, factor) {
    const current = readViewBox(svg);
    if (!current || !baseViewBox) return false;
    const minWidth = baseViewBox.w / MAX_ZOOM;
    const maxWidth = baseViewBox.w;
    const newWidth = Math.min(Math.max(current.w * factor, minWidth), maxWidth);
    const newHeight = newWidth * (baseViewBox.h / baseViewBox.w);
    const scale = newWidth / current.w;
    const next = {
        x: anchorX - (anchorX - current.x) * scale,
        y: anchorY - (anchorY - current.y) * scale,
        w: newWidth,
        h: newHeight,
    };
    writeViewBox(svg, clampViewBox(next));
    return true;
}

// 把无人机的 AirSim NED 位置换算成这张地图用的 CARLA 坐标。
function updateDroneCarlaPosition(drone) {
    if (!drone || !drone.position || !mapOffset) {
        droneCarlaPosition = null;
        return false;
    }
    const x = Number(drone.position.x);
    const y = Number(drone.position.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
        droneCarlaPosition = null;
        return false;
    }
    droneCarlaPosition = [x - mapOffset.x, y - mapOffset.y];
    return true;
}

// 在地图上标出无人机当前位置（蓝点）——点选起点时好知道它离你点多远。
function renderDroneMarker() {
    const svg = document.getElementById('routeMapSvg');
    if (!svg) return;
    svg.querySelectorAll('.drone-marker').forEach((el) => el.remove());
    if (!viewTransform || !droneCarlaPosition) return;
    const [cx, cy] = carlaToSvg(droneCarlaPosition);
    const group = createSvgElement('g', {});
    group.classList.add('drone-marker');
    const halo = createSvgElement('circle', { cx: cx.toFixed(2), cy: cy.toFixed(2), r: 7 });
    halo.setAttribute('fill', 'rgba(37, 99, 235, .16)');
    group.appendChild(halo);
    const dot = createSvgElement('circle', { cx: cx.toFixed(2), cy: cy.toFixed(2), r: 3.2 });
    dot.setAttribute('fill', '#2563eb');
    dot.setAttribute('stroke', '#ffffff');
    dot.setAttribute('stroke-width', '1.6');
    dot.setAttribute('vector-effect', 'non-scaling-stroke');
    group.appendChild(dot);
    const label = createSvgElement('text', { x: (cx + 7).toFixed(2), y: (cy - 6).toFixed(2) });
    label.textContent = '无人机';
    label.classList.add('drone-marker-label');
    group.appendChild(label);
    svg.appendChild(group);
}

// 绘制自定义规划叠加层：规划出的绿色折线 + 用户点击的序号选点。
function renderPlanningOverlay(elements) {
    elements.svg.querySelectorAll('.planning-overlay').forEach((el) => el.remove());

    if (plannedPolyline && plannedPolyline.length >= 2) {
        const line = createSvgElement('polyline', {
            points: plannedPolyline
                .map((p) => carlaToSvg(p).map((v) => v.toFixed(2)).join(','))
                .join(' '),
        });
        line.setAttribute('fill', 'none');
        line.setAttribute('stroke', '#10b981');
        line.setAttribute('stroke-width', '3');
        line.setAttribute('stroke-linejoin', 'round');
        line.setAttribute('opacity', '0.9');
        line.classList.add('planning-overlay');
        elements.svg.appendChild(line);
    }

    selectedPoints.forEach((point, index) => {
        const [cx, cy] = carlaToSvg(point);
        const isStart = index === 0;
        const isEnd = index === selectedPoints.length - 1;
        const color = isStart ? '#22c55e' : (isEnd ? '#ef4444' : '#3b82f6');
        const circle = createSvgElement('circle', { cx: cx.toFixed(2), cy: cy.toFixed(2), r: 3 });
        circle.setAttribute('fill', color);
        circle.setAttribute('stroke', '#ffffff');
        circle.setAttribute('stroke-width', '1.5');
        circle.classList.add('planning-overlay');
        elements.svg.appendChild(circle);

        const label = createSvgElement('text', {
            x: cx.toFixed(2),
            y: (cy - 5).toFixed(2),
            'text-anchor': 'middle',
        });
        label.setAttribute('font-size', '4');
        label.setAttribute('fill', '#f8fafc');
        label.setAttribute('stroke', '#172235');
        label.setAttribute('stroke-width', '0.8');
        label.classList.add('planning-overlay');
        label.textContent = String(index + 1);
        elements.svg.appendChild(label);
    });
}

function renderTopology(data, route, elements) {
    const roads = Array.isArray(data.roads) ? data.roads : [];
    const routePoints = route && Array.isArray(route.waypoints)
        ? route.waypoints.map((point) => [Number(point.x), Number(point.y)])
        : [];
    // 记录 AirSim -> CARLA 的偏移，用于把无人机的实时位置画到这张图上。
    mapOffset = route && route.offset && Number.isFinite(Number(route.offset.x))
        ? { x: Number(route.offset.x), y: Number(route.offset.y) }
        : null;
    const bounds = data.bounds || {};
    const basePoints = roads.flatMap((road) => Array.isArray(road.points) ? road.points : []);
    const allPoints = [...basePoints, ...routePoints];
    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    allPoints.forEach(([x, y]) => {
        const pointX = Number(x);
        const pointY = Number(y);
        if (!Number.isFinite(pointX) || !Number.isFinite(pointY)) return;
        minX = Math.min(minX, pointX);
        maxX = Math.max(maxX, pointX);
        minY = Math.min(minY, pointY);
        maxY = Math.max(maxY, pointY);
    });
    if (!Number.isFinite(minX)) {
        minX = Number(bounds.min_x) || 0;
        maxX = Number(bounds.max_x) || minX + 1;
        minY = Number(bounds.min_y) || 0;
        maxY = Number(bounds.max_y) || minY + 1;
    }
    const width = Math.max(1, maxX - minX);
    const height = Math.max(1, maxY - minY);
    const padding = Math.max(10, Math.max(width, height) * 0.03);

    // 记录坐标变换，供点击反算使用。
    viewTransform = { minX, maxY, padding, width, height };

    const toSvgPoint = ([x, y]) => [
        Number(x) - minX + padding,
        maxY - Number(y) + padding,
    ];

    elements.svg.replaceChildren();
    // 记录自适应 fit 视图；每次读取地图都回到该视图，缩放/平移后可一键重置。
    baseViewBox = { x: 0, y: 0, w: width + padding * 2, h: height + padding * 2 };
    writeViewBox(elements.svg, baseViewBox);
    elements.svg.setAttribute('preserveAspectRatio', 'xMidYMid meet');
    const defs = createSvgElement('defs');
    defs.appendChild(createRouteArrowMarker());
    elements.svg.appendChild(defs);

    // 道路网络保持统一弱化颜色，避免与实际巡航轨迹混淆。
    roads.forEach((road) => {
        const points = Array.isArray(road.points) ? road.points : [];
        if (points.length < 2) return;
        const polyline = createSvgElement('polyline', {
            points: points.map((point) => toSvgPoint(point).map((value) => value.toFixed(2)).join(',')).join(' '),
            'data-road-id': road.road_id ?? '',
        });
        polyline.classList.add('road-network');
        elements.svg.appendChild(polyline);
    });

    if (routePoints.length >= 2) {
        const routeLine = createSvgElement('polyline', {
            points: routePoints.map((point) => toSvgPoint(point).map((value) => value.toFixed(2)).join(',')).join(' '),
        });
        routeLine.classList.add('cruise-route');
        elements.svg.appendChild(routeLine);

        const [startX, startY] = toSvgPoint(routePoints[0]);
        const [endX, endY] = toSvgPoint(routePoints[routePoints.length - 1]);
        const markerRadius = Math.max(3.5, Math.max(width, height) * 0.012);

        const startMarker = createSvgElement('polygon', {
            points: createStarPoints(startX, startY, markerRadius * 1.8, markerRadius * .8),
        });
        startMarker.classList.add('route-start-marker');
        elements.svg.appendChild(startMarker);
        const startLabel = createSvgElement('text', {
            x: startX - markerRadius * 1.5,
            y: startY + markerRadius * 2.5,
            'text-anchor': 'end',
        });
        startLabel.classList.add('route-marker-label');
        startLabel.textContent = '起点';
        elements.svg.appendChild(startLabel);

        const endMarker = createSvgElement('circle', { cx: endX, cy: endY, r: markerRadius * .9 });
        endMarker.classList.add('route-end-marker');
        elements.svg.appendChild(endMarker);
        const endLabel = createSvgElement('text', {
            x: endX + markerRadius * 1.5,
            y: endY - markerRadius * 1.8,
            'text-anchor': 'start',
        });
        endLabel.classList.add('route-marker-label');
        endLabel.textContent = '终点';
        elements.svg.appendChild(endLabel);
    }

    // 叠加自定义规划层（选点 + 规划折线）。
    renderPlanningOverlay(elements);

    elements.stage.classList.toggle('has-topology', roads.length > 0);
    elements.empty.hidden = roads.length > 0;
    elements.currentMap.textContent = data.map || '--';
    elements.roadCount.textContent = String(data.lane_count ?? roads.length);
    elements.routeWaypointCount.textContent = route ? String(route.waypoint_count ?? routePoints.length) : '--';
    const alignmentLabels = {
        configured_offset: '已校准',
        auto_matched: '自动对齐',
        identity_fallback: '对齐失败',
        identity: '原点一致',
    };
    elements.alignment.textContent = route
        ? (alignmentLabels[route.alignment] || route.alignment || '未对齐')
        : '无固定航线';
    elements.alignment.title = route && Number.isFinite(Number(route.alignment_error))
        ? `道路匹配误差 ${Number(route.alignment_error).toFixed(1)}m`
        : '';
    elements.saveStatus.textContent = route ? '固定航线已加载' : '暂无固定航线';
    // 地图一画好就把无人机位置标出来（位置由巡航状态轮询持续刷新）。
    renderDroneMarker();
}

export function initRoutePlanning() {
    const select = document.getElementById('routeMapSelect');
    const loadButton = document.getElementById('routeLoadMapBtn');
    const stage = document.getElementById('routeMapPreview');
    const svg = document.getElementById('routeMapSvg');
    const empty = document.getElementById('routeMapEmpty');
    const emptyTitle = document.getElementById('routeMapEmptyTitle');
    const emptyText = document.getElementById('routeMapEmptyText');
    const stateBadge = document.querySelector('.route-page-state');
    const currentMap = document.getElementById('routeCurrentMap');
    const roadCount = document.getElementById('routeRoadCount');
    const routeWaypointCount = document.getElementById('routeWaypointCount');
    const alignment = document.getElementById('routeAlignmentStatus');
    const saveStatus = document.getElementById('routeSaveStatus');
    const undoBtn = document.getElementById('routeUndoBtn');
    const clearBtn = document.getElementById('routeClearBtn');
    const saveBtn = document.getElementById('routeSaveBtn');
    const pointList = document.getElementById('routePointList');
    const cruiseSpeedInput = document.getElementById('routeCruiseSpeed');
    const cruiseStartBtn = document.getElementById('routeCruiseStartBtn');
    const cruisePauseBtn = document.getElementById('routeCruisePauseBtn');
    const cruiseLandBtn = document.getElementById('routeCruiseLandBtn');
    const cruiseStatus = document.getElementById('routeCruiseStatus');
    const modeFixedBtn = document.getElementById('routeModeFixedBtn');
    const modeCustomBtn = document.getElementById('routeModeCustomBtn');
    const cruiseStateEl = document.getElementById('routeCruiseState');
    const dronePosEl = document.getElementById('routeDronePosition');
    const pathProgressEl = document.getElementById('routePathProgress');
    const resetViewBtn = document.getElementById('routeResetViewBtn');
    const coordinateReadout = document.getElementById('routeMapCoordinates');

    if (!select || !loadButton || !stage || !svg || !empty || !emptyTitle || !emptyText
        || !stateBadge || !currentMap || !roadCount || !routeWaypointCount || !alignment || !saveStatus) {
        return;
    }

    const elements = {
        select, loadButton, stage, svg, empty, emptyTitle, emptyText,
        stateBadge, currentMap, roadCount, routeWaypointCount, alignment, saveStatus,
    };

    // 启动请求进行中标记：轮询不得在此期间覆盖“开始”按钮的禁用态。
    let startingCruise = false;
    // 拖动平移状态；suppressNextClick 防止"拖完地图顺手加了一个航点"。
    let panState = null;
    let suppressNextClick = false;

    function setState(text, kind = '') {
        stateBadge.textContent = text;
        stateBadge.classList.remove('is-loading', 'is-ready', 'is-error');
        if (kind) stateBadge.classList.add(`is-${kind}`);
    }

    function updateCruiseStatus() {
        if (!cruiseStatus) return;
        // 画了线但没保存：启动巡航会走固定航线，这里必须说清楚，否则用户以为在飞自己画的线。
        if (selectedPoints.length >= 2 && pointsDirty) {
            cruiseStatus.textContent = '⚠️ 你画的路线还没保存：现在起飞走的是"固定航线"。请先点「💾 保存自定义路线」';
            cruiseStatus.style.color = '#b91c1c';
            return;
        }
        cruiseStatus.style.color = '#64748b';
        const hasCustom = Boolean(savedRouteId && savedRouteMap === select.value);
        if (state.routeMode === 'custom' && hasCustom) {
            cruiseStatus.textContent = `将执行自定义路线 ${savedRouteId}（${savedRouteMap}）`;
        } else if (state.routeMode === 'custom') {
            cruiseStatus.textContent = '当前地图还没有已保存的自定义航线，启动时将改用固定航线';
        } else {
            cruiseStatus.textContent = '将执行当前地图的固定航线';
        }
    }

    function setRouteMode(mode) {
        state.routeMode = mode === 'custom' ? 'custom' : 'fixed';
        if (modeFixedBtn) modeFixedBtn.classList.toggle('active', state.routeMode === 'fixed');
        if (modeCustomBtn) modeCustomBtn.classList.toggle('active', state.routeMode === 'custom');
        updateCruiseStatus();
    }

    // 保证 CARLA 已连接且当前世界地图与所选地图一致；
    // 拓扑缓存允许“预览”跳过切图，但规划/保存/巡航必须基于与界面一致的世界地图。
    async function ensureCarlaMap(targetMap) {
        let status;
        try {
            status = await getStatus();
        } catch (statusError) {
            throw new Error('无法连接后端服务，请确认后端已启动');
        }
        if (!status.connected) {
            throw new Error('未连接到 CARLA-Air，请先在仿真器配置页启动');
        }
        if (shortMapName(status.map) !== shortMapName(targetMap)) {
            showNotification(`正在切换地图到 ${targetMap}...`, 'info');
            const switchResult = await loadMap(targetMap);
            if (!switchResult.success) {
                throw new Error(switchResult.error || `地图切换到 ${targetMap} 失败`);
            }
        }
        return true;
    }

    function updatePointList() {
        if (!pointList) return;
        if (selectedPoints.length === 0) {
            pointList.innerHTML = '尚未创建航点。点击地图添加起点 / 途经点 / 终点。';
            return;
        }
        pointList.innerHTML = selectedPoints.map((p, i) => {
            const role = i === 0 ? '起点' : (i === selectedPoints.length - 1 ? '终点' : '途经点');
            return `<div style="display:flex;justify-content:space-between;gap:8px;font-size:11px;color:#334155;padding:3px 0;border-bottom:1px solid #eef2f9;">
                <span>${role} ${i + 1}</span>
                <span style="font-family:Consolas,monospace;color:#64748b;">(${p[0].toFixed(1)}, ${p[1].toFixed(1)})</span>
            </div>`;
        }).join('') + startDistanceHint();
    }

    // 显示"无人机距起点多远"，顺带提示 200m 的连接段上限，避免点了才报错。
    function startDistanceHint() {
        if (!droneCarlaPosition || selectedPoints.length === 0) return '';
        const first = selectedPoints[0];
        const distance = Math.hypot(first[0] - droneCarlaPosition[0], first[1] - droneCarlaPosition[1]);
        if (distance > 200) {
            return `<div style="margin-top:8px;font-size:11px;line-height:1.5;color:#b91c1c;background:#fef2f2;border-radius:8px;padding:6px 8px;">
                ⚠️ 无人机距起点 ${distance.toFixed(0)} m，超过 200 m 上限，无法起飞。<br>请把起点选在无人机（蓝点）附近。
            </div>`;
        }
        const note = distance > 35
            ? `，前 ${distance.toFixed(0)} m 会直线飞过去（连接段，不沿路）`
            : '，就在起点附近，可以直接起飞';
        return `<div style="margin-top:8px;font-size:11px;line-height:1.5;color:#0369a1;background:#f0f9ff;border-radius:8px;padding:6px 8px;">
            🛸 无人机距起点 ${distance.toFixed(0)} m${note}
        </div>`;
    }

    function clearPreview(message = '点击“读取地图”后，这里将显示道路网络和固定巡航路线。') {
        selectedPoints = [];
        plannedPolyline = null;
        viewTransform = null;
        baseViewBox = null;
        resetCoordinateReadout();
        updatePointList();
        svg.replaceChildren();
        stage.classList.remove('has-topology');
        empty.hidden = false;
        emptyTitle.textContent = '等待地图数据';
        emptyText.textContent = message;
        roadCount.textContent = '0';
        routeWaypointCount.textContent = '--';
        alignment.textContent = '待读取';
        saveStatus.textContent = '未保存';
    }

    async function planLive() {
        if (selectedPoints.length < 2) {
            plannedPolyline = null;
            routeWaypointCount.textContent = '--';
            renderPlanningOverlay(elements);
            return;
        }
        try {
            // 世界地图必须与界面所选地图一致，规划结果才能与预览路网对应。
            await ensureCarlaMap(select.value);
            const result = await planRoute(selectedPoints);
            if (result.success) {
                plannedPolyline = (result.polyline || []).map((p) => [p.x, p.y, p.z]);
                routeWaypointCount.textContent = String(result.node_count ?? plannedPolyline.length);
                if (result.warnings && result.warnings.length) {
                    showNotification(result.warnings.join('；'), 'warning');
                }
            } else {
                plannedPolyline = null;
                routeWaypointCount.textContent = '规划失败';
                showNotification('规划失败: ' + (result.error || '未知错误'), 'error');
            }
        } catch (error) {
            plannedPolyline = null;
            routeWaypointCount.textContent = '规划失败';
            showNotification('规划失败: ' + error.message, 'error');
        }
        renderPlanningOverlay(elements);
    }

    async function saveRoute() {
        if (selectedPoints.length < 2) {
            showNotification('请至少点击起点和终点两个点', 'warning');
            return;
        }
        const altitude = parseFloat(document.getElementById('routeCruiseAltitude')?.value) || 45;
        saveStatus.textContent = '保存中...';
        try {
            // 保存的航线属于 CARLA 世界地图；先保证与所选地图一致，避免把 A 图的航线存成 B 图。
            await ensureCarlaMap(select.value);
            const result = await saveCustomRoute(selectedPoints, altitude);
            if (result.success) {
                savedRouteId = result.route_id;
                savedRouteMap = result.map ? shortMapName(result.map) : select.value;
                state.customRouteId = savedRouteId;
                state.customRouteMap = savedRouteMap;
                saveStatus.textContent = `已保存 (${result.waypoint_count} 航点)`;
                // 保存成功后自动切换到“自定义路线”模式，巡航/采集默认执行刚保存的航线。
                pointsDirty = false;
                setRouteMode('custom');
                showNotification(`自定义航线已保存: ${result.route_id}`, 'success');
            } else {
                saveStatus.textContent = '保存失败';
                showNotification('保存失败: ' + (result.error || '未知错误'), 'error');
            }
        } catch (error) {
            saveStatus.textContent = '保存失败';
            showNotification('保存失败: ' + error.message, 'error');
        }
    }

    async function startCruise() {
        // 画了线但没保存：先问清楚按哪条航线飞，避免"以为在飞自己画的线，其实飞的是固定航线"。
        if (selectedPoints.length >= 2 && pointsDirty) {
            const saveAndFly = window.confirm(
                `你在图上画了 ${selectedPoints.length} 个点，但还没有保存成自定义航线。\n\n` +
                '点「确定」：先保存这条自定义航线，并按它起飞\n' +
                '点「取消」：不保存，按当前地图的固定航线起飞'
            );
            if (saveAndFly) {
                await saveRoute();
                if (!savedRouteId) {
                    showNotification('自定义航线保存失败，本次仍按固定航线起飞', 'warning');
                } else {
                    showNotification(`已保存自定义航线 ${savedRouteId}，按它起飞`, 'success');
                }
            }
        }
        if (cruiseStartBtn) {
            cruiseStartBtn.disabled = true;
            cruiseStartBtn.textContent = '启动中...';
        }
        startingCruise = true;
        try {
            // 与“生成数据集”流程保持一致：先确保 CARLA 就绪且当前地图与所选地图一致，再启动巡航。
            await ensureCarlaMap(select.value);

            const altitude = parseFloat(document.getElementById('routeCruiseAltitude')?.value) || 45;
            const speed = parseFloat(cruiseSpeedInput?.value) || 8;
            // 同步共享状态与采集页输入，保持两页巡航参数一致。
            state.cruiseSpeed = speed;
            state.cruiseAltitude = altitude;
            const collectAltitudeInput = document.getElementById('cruiseAltitudeInput');
            const collectSpeedInput = document.getElementById('cruiseSpeedInput');
            if (collectAltitudeInput) collectAltitudeInput.value = altitude;
            if (collectSpeedInput) collectSpeedInput.value = speed;
            const hasCustom = Boolean(savedRouteId && savedRouteMap === select.value);
            let routeId = null;
            if (state.routeMode === 'custom') {
                if (hasCustom) {
                    routeId = savedRouteId;
                } else {
                    showNotification('当前地图还没有已保存的自定义航线，改用固定航线', 'warning');
                }
            }
            const result = await droneCruiseStart(speed, altitude, routeId);
            showNotification(
                `自动巡航已启动（${result.map_name}，${result.waypoints_count} 航点，${routeId ? '自定义路线' : '固定航线'}）`,
                'success'
            );
        } catch (error) {
            showNotification('巡航启动失败: ' + error.message, 'error');
        } finally {
            startingCruise = false;
            if (cruiseStartBtn) {
                cruiseStartBtn.disabled = false;
                cruiseStartBtn.textContent = '✈️ 开始自动巡航';
            }
            // 启动失败时立即刷新一次真实状态，避免按钮停在错误的可用/禁用态。
            pollCruiseState();
        }
    }

    async function pauseCruise() {
        try {
            const result = await droneCruiseStop();
            showNotification(result.message || '巡航已暂停', 'success');
        } catch (error) {
            showNotification('暂停失败: ' + error.message, 'error');
        }
    }

    async function landCruise() {
        try {
            const result = await droneLand();
            showNotification(result.message || '无人机已降落', 'success');
        } catch (error) {
            showNotification('降落失败: ' + error.message, 'error');
        }
    }

    // 巡航状态文案（与后端 control_state 对应）。
    const cruiseStateLabels = {
        manual: '手动控制',
        preparing: '准备中',
        loading_fixed_route: '加载固定航线',
        loading_custom_route: '加载自定义航线',
        taking_off: '起飞中',
        auto_cruise: '自动巡航中',
        paused: '已暂停',
        manual_override: '已交还手动',
        landing: '降落中',
        landing_error: '降落失败',
        landed: '已降落',
    };
    // 这些状态下“开始自动巡航”按钮应禁用，避免重复提交飞行任务。
    const cruiseBusyStates = new Set([
        'preparing', 'loading_fixed_route', 'loading_custom_route',
        'taking_off', 'auto_cruise', 'landing',
    ]);

    async function pollCruiseState() {
        const page = document.getElementById('page-route-planning');
        // 页面未激活时不打扰后端；切回本页时下一次轮询会立即刷新。
        if (page && !page.classList.contains('active-page')) return;
        try {
            const drone = await droneStatus();
            // 把无人机实时位置画到地图上，并刷新"距起点多远"的提示。
            if (updateDroneCarlaPosition(drone)) {
                renderDroneMarker();
                if (selectedPoints.length > 0) updatePointList();
            }
            if (cruiseStateEl) {
                cruiseStateEl.textContent = cruiseStateLabels[drone.control_state] || drone.control_state || '未知';
            }
            if (dronePosEl) {
                dronePosEl.textContent = drone.position
                    ? `(${drone.position.x}, ${drone.position.y}, ${drone.position.z})`
                    : '--';
            }
            if (pathProgressEl) {
                pathProgressEl.textContent = (drone.cruising && drone.total_waypoints > 0)
                    ? `${drone.waypoint_index} / ${drone.total_waypoints}`
                    : '--';
            }
            // 巡航/起降进行中禁用“开始”按钮；启动请求进行中（startingCruise）不抢占按钮状态。
            if (cruiseStartBtn && !startingCruise) {
                cruiseStartBtn.disabled = drone.cruising || cruiseBusyStates.has(drone.control_state);
            }
        } catch (error) {
            // 后端未启动等场景由既有健康检查与通知处理，这里静默即可。
        }
    }

    function undoPoint() {
        if (selectedPoints.length === 0) return;
        selectedPoints.pop();
        pointsDirty = true;
        updatePointList();
        renderPlanningOverlay(elements);
        updateCruiseStatus();
        planLive();
    }

    function clearPoints() {
        selectedPoints = [];
        plannedPolyline = null;
        pointsDirty = false;
        routeWaypointCount.textContent = '--';
        saveStatus.textContent = '未保存';
        updatePointList();
        renderPlanningOverlay(elements);
        updateCruiseStatus();
    }

    // ---- 地图交互：光标坐标读数 / 滚轮缩放 / 拖动平移 / 重置视图 ----

    function updateCoordinateReadout(clientX, clientY) {
        if (!coordinateReadout) return;
        const point = svgClientToCarla(clientX, clientY, svg);
        coordinateReadout.textContent = point
            ? `X: ${point[0].toFixed(1)}   Y: ${point[1].toFixed(1)}   Z: --`
            : 'X: --   Y: --   Z: --';
    }

    function resetCoordinateReadout() {
        if (coordinateReadout) coordinateReadout.textContent = 'X: --   Y: --   Z: --';
    }

    function onWheel(event) {
        if (!viewTransform || !baseViewBox) return;
        // 阻止默认滚动，否则滚轮会带着整个页面一起滚。
        event.preventDefault();
        const anchor = svgClientToBase(event.clientX, event.clientY, svg);
        if (!anchor) return;
        // 向上滚（deltaY < 0）放大，向下滚缩小。
        const factor = event.deltaY < 0 ? 1 / ZOOM_STEP : ZOOM_STEP;
        if (zoomView(svg, anchor.x, anchor.y, factor)) {
            updateCoordinateReadout(event.clientX, event.clientY);
        }
    }

    function onPointerDown(event) {
        if (!viewTransform || !baseViewBox) return;
        if (event.button !== 0) return;           // 只用左键拖动
        const startViewBox = readViewBox(svg);
        if (!startViewBox) return;
        // 缩放/平移不改变缩放比例，所以在按下时抓一次"每个屏幕像素对应多少 base 单位"即可。
        // 注意：不能在移动过程中"用当前 viewBox 反算 base 再相减"——那样参照系会跟着视图一起动。
        const ctm = typeof svg.getScreenCTM === 'function' ? svg.getScreenCTM() : null;
        const unitsPerPxX = ctm && ctm.a ? 1 / ctm.a : 1;
        const unitsPerPxY = ctm && ctm.d ? 1 / ctm.d : 1;
        suppressNextClick = false;
        panState = {
            pointerId: event.pointerId,
            startClientX: event.clientX,
            startClientY: event.clientY,
            startViewBox,
            unitsPerPxX,
            unitsPerPxY,
            moved: false,
        };
        if (typeof svg.setPointerCapture === 'function') {
            try { svg.setPointerCapture(event.pointerId); } catch (error) { /* 忽略 */ }
        }
    }

    function onPointerMove(event) {
        if (!viewTransform) return;
        if (!panState || event.pointerId !== panState.pointerId) {
            // 只是移动鼠标：更新坐标读数，方便对准位置点选。
            updateCoordinateReadout(event.clientX, event.clientY);
            return;
        }
        const dxPx = event.clientX - panState.startClientX;
        const dyPx = event.clientY - panState.startClientY;
        if (!panState.moved && Math.hypot(dxPx, dyPx) > DRAG_THRESHOLD_PX) {
            panState.moved = true;
            stage.classList.add('is-panning');
        }
        if (!panState.moved) return;

        // 往右拖 = 看到左边更多 = viewBox.x 变小。
        writeViewBox(svg, clampViewBox({
            x: panState.startViewBox.x - dxPx * panState.unitsPerPxX,
            y: panState.startViewBox.y - dyPx * panState.unitsPerPxY,
            w: panState.startViewBox.w,
            h: panState.startViewBox.h,
        }));
    }

    function onPointerUp(event) {
        if (!panState || event.pointerId !== panState.pointerId) return;
        if (panState.moved) suppressNextClick = true;   // 拖动结束不要顺手加航点
        panState = null;
        stage.classList.remove('is-panning');
        if (typeof svg.releasePointerCapture === 'function') {
            try { svg.releasePointerCapture(event.pointerId); } catch (error) { /* 忽略 */ }
        }
    }

    function resetView() {
        if (!baseViewBox) {
            showNotification('请先点击“读取地图”加载道路网络', 'warning');
            return;
        }
        writeViewBox(svg, { ...baseViewBox });
    }

    function onMapClick(event) {
        if (suppressNextClick) {
            suppressNextClick = false;
            return;
        }
        if (!viewTransform) {
            showNotification('请先点击“读取地图”加载道路网络', 'warning');
            return;
        }
        const point = svgClientToCarla(event.clientX, event.clientY, svg);
        if (!point) return;
        selectedPoints.push(point);
        pointsDirty = true;
        updatePointList();
        renderPlanningOverlay(elements);
        updateCruiseStatus();
        planLive();
    }

    async function loadSelectedTopology() {
        const selectedMap = select.value;
        loadButton.disabled = true;
        loadButton.textContent = '读取中...';
        setState('正在读取地图', 'loading');
        empty.hidden = false;
        emptyTitle.textContent = '正在读取道路网络';
        emptyText.textContent = '正在读取道路和固定航线数据，请稍候。';
        // 切图后重置之前的自定义选点。
        selectedPoints = [];
        plannedPolyline = null;
        updatePointList();

        try {
            let topology;
            try {
                // 已有道路缓存时，不重新查询或切换 CARLA 地图。
                topology = await getMapTopology(selectedMap, 3);
            } catch (topologyError) {
                // 只有首次生成目标地图时，才切换 CARLA 并建立道路缓存。
                const status = await getStatus();
                if (!status.connected || shortMapName(status.map) === shortMapName(selectedMap)) {
                    throw topologyError;
                }
                const switchResult = await loadMap(selectedMap);
                if (!switchResult.success) {
                    throw new Error(switchResult.error || `地图切换到 ${selectedMap} 失败`);
                }
                topology = await getMapTopology(selectedMap, 3);
            }

            if (!topology.success) {
                throw new Error(topology.error || '道路拓扑读取失败');
            }

            let fixedRoute = null;
            try {
                fixedRoute = await getFixedRoute(selectedMap);
            } catch (routeError) {
                // 没有固定航线的资源地图仍可显示道路网络。
                console.info(`[Route] ${selectedMap} 没有固定巡航航线: ${routeError.message}`);
            }

            renderTopology(topology, fixedRoute, elements);
            const fromCache = topology.source === 'fixed_cache';
            setState(fromCache ? `${topology.map} 固定地图已加载` : `${topology.map} 已生成并缓存`, 'ready');
            showNotification(
                fixedRoute
                    ? `已显示 ${topology.map} 的道路网络和固定巡航路线`
                    : `已显示 ${topology.map} 的道路网络，暂无固定巡航路线`,
                'success'
            );
        } catch (error) {
            clearPreview(error.message || '无法读取地图数据');
            setState('地图读取失败', 'error');
            emptyTitle.textContent = '地图读取失败';
            showNotification(`地图预览失败: ${error.message || '请检查 CARLA-Air'}`, 'error');
        } finally {
            loadButton.disabled = false;
            loadButton.textContent = '读取地图';
        }
    }

    select.addEventListener('change', () => {
        clearPreview('地图已切换，请点击“读取地图”读取道路网络和固定巡航路线。');
        setState('等待读取');
        currentMap.textContent = select.value;
        updateCruiseStatus();
    });
    loadButton.addEventListener('click', loadSelectedTopology);
    svg.addEventListener('click', onMapClick);
    // 滚轮缩放：passive:false 才能 preventDefault 阻止页面跟着滚。
    svg.addEventListener('wheel', onWheel, { passive: false });
    svg.addEventListener('pointerdown', onPointerDown);
    svg.addEventListener('pointermove', onPointerMove);
    svg.addEventListener('pointerup', onPointerUp);
    svg.addEventListener('pointercancel', onPointerUp);
    svg.addEventListener('pointerleave', () => {
        if (!panState) resetCoordinateReadout();
    });
    if (resetViewBtn) resetViewBtn.addEventListener('click', resetView);
    if (undoBtn) undoBtn.addEventListener('click', undoPoint);
    if (clearBtn) clearBtn.addEventListener('click', clearPoints);
    if (saveBtn) saveBtn.addEventListener('click', saveRoute);
    if (modeFixedBtn) modeFixedBtn.addEventListener('click', () => setRouteMode('fixed'));
    if (modeCustomBtn) modeCustomBtn.addEventListener('click', () => setRouteMode('custom'));
    if (cruiseStartBtn) cruiseStartBtn.addEventListener('click', startCruise);
    if (cruisePauseBtn) cruisePauseBtn.addEventListener('click', pauseCruise);
    if (cruiseLandBtn) cruiseLandBtn.addEventListener('click', landCruise);
    // 按共享状态初始化模式开关，并启动巡航状态轮询（仅本页激活时刷新）。
    setRouteMode(state.routeMode);
    setInterval(pollCruiseState, 2000);
    pollCruiseState();
}
