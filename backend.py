import sys
import json
import time
import math
import argparse
import subprocess
import os
import re
import threading
import ctypes
from pathlib import Path
from flask import Flask, request, jsonify
from flask_cors import CORS

try:
    from sensor_manager import (
        spawn_rgb_camera,
        spawn_depth_camera,
        spawn_semantic_camera,
        spawn_instance_camera,
        spawn_optical_flow_camera,
        spawn_normals_camera,
        spawn_lidar,
        spawn_semantic_lidar,
        spawn_radar,
        spawn_gnss,
        spawn_imu,
        destroy_sensors,
        start_listeners,
        generate_airsim_settings,
        collect_airsim_frame,
    )
except Exception as sensor_import_error:
    # 缺少 carla/airsim 等依赖时，地图预览等文件级功能仍可用；采集相关接口返回明确错误。
    print(f"警告: 无法导入 sensor_manager 模块（预览功能不受影响）: {sensor_import_error}")
    def _sensor_module_missing(*args, **kwargs):
        raise RuntimeError('传感器模块未加载：需要完整的 CARLA/AirSim 环境')
    spawn_rgb_camera = _sensor_module_missing
    spawn_depth_camera = _sensor_module_missing
    spawn_semantic_camera = _sensor_module_missing
    spawn_instance_camera = _sensor_module_missing
    spawn_optical_flow_camera = _sensor_module_missing
    spawn_normals_camera = _sensor_module_missing
    spawn_lidar = _sensor_module_missing
    spawn_semantic_lidar = _sensor_module_missing
    spawn_radar = _sensor_module_missing
    spawn_gnss = _sensor_module_missing
    spawn_imu = _sensor_module_missing
    destroy_sensors = lambda *args, **kwargs: None
    start_listeners = _sensor_module_missing
    generate_airsim_settings = _sensor_module_missing
    collect_airsim_frame = _sensor_module_missing

def get_carla_air_dir():
    '''在不写死路径的情况下找到存放carlaAir文件的目录'''
    appdata = os.environ.get('APPDATA', '')
    config_file = os.path.join(appdata, 'drone-dataset-platform', 'config.json')
    print(f"查找配置文件: {config_file}")

    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
                carla_path = config.get('carlaAirPath')
                if carla_path and os.path.exists(carla_path):
                    print(f"[OK] 从配置文件读取 Carla-Air: {carla_path}")
                    return carla_path
                else:
                    print(f"[FAIL] 配置中的路径无效: {carla_path}")
        except Exception as e:
            print(f"读取配置文件失败: {e}")
    else:
        print(f"[FAIL] 配置文件不存在: {config_file}")

    env_path = os.environ.get('CARLA_AIR_PATH')
    if env_path and os.path.exists(env_path):
        print(f"[OK] 从环境变量获取 Carla-Air: {env_path}")
        return env_path

    if len(sys.argv) > 1:
        path = sys.argv[1]
        if os.path.exists(path):
            print(f"[OK] 从命令行参数获取 Carla-Air: {path}")
            return path

    print("[ERROR] 未找到 Carla-Air 路径")
    return None

carla_air_dir = get_carla_air_dir()

if carla_air_dir:
    carla_api_dist = os.path.join(carla_air_dir, 'PythonAPI', 'carla', 'dist')
    if os.path.exists(carla_api_dist):
        whl_files = [f for f in os.listdir(carla_api_dist) if f.endswith('.whl')]
        if whl_files:
            CARLA_API_PATH = os.path.join(carla_api_dist, whl_files[0])
            sys.path.append(CARLA_API_PATH)
            print(f"已添加 CARLA API 路径: {CARLA_API_PATH}")
        else:
            print(f"警告: CARLA API 目录下没有 .whl 文件: {carla_api_dist}")
    else:
        print(f"警告: CARLA API 路径不存在: {carla_api_dist}")

try:
    import airsim
    print("成功导入 airsim 模块")
except ImportError as e:
    print(f"错误: 无法导入 airsim 模块: {e}")
    airsim = None

try:
    from road_cruise import (
        CruiseRoute,
        RoadCruiseController,
        load_fixed_cruise_route,
        nearest_route_index,
        normalize_map_name,
    )
    print("成功导入 road_cruise 模块")
except ImportError as e:
    print(f"警告: 无法导入 road_cruise 模块: {e}")
    CruiseRoute = None
    RoadCruiseController = None
    load_fixed_cruise_route = None
    nearest_route_index = None
    normalize_map_name = lambda name: _short_map_name(name)

try:
    from custom_route import (
        plan_route_carla,
        carla_polyline_to_airsim_route,
        save_custom_route,
        load_custom_cruise_route,
        list_custom_routes,
        prepend_start_connector,
    )
    print("成功导入 custom_route 模块")
except ImportError as e:
    print(f"警告: 无法导入 custom_route 模块: {e}")
    plan_route_carla = None
    carla_polyline_to_airsim_route = None
    save_custom_route = None
    load_custom_cruise_route = None
    list_custom_routes = None
    prepend_start_connector = None

try:
    import carla
    print("成功导入 carla 模块")
except ImportError as e:
    print(f"错误: 无法导入 carla 模块: {e}")
    print(f"请确认 Carla-Air 路径正确: {carla_air_dir}")
    carla = None

app = Flask(__name__)
CORS(app)

client = None
world = None
drone_actor = None
active_sensors = []
sensor_save_dir = None

# AirSim / 无人机巡航全局变量
airsim_client = None
airsim_connected = False
cruise_controller = None
cruise_thread = None
cruise_running = False
# 飞控任务只在启动时提交一次；采集线程和巡航线程都不再重复发送移动命令。
cruise_state = 'manual'
cruise_altitude = 45.0
cruise_lock = threading.RLock()
# Unreal 进程的键盘输入不能由 Python 直接屏蔽；检测到飞行控制按键后，
# 将其解释为用户主动退出自动巡航，并通过状态接口通知前端。
manual_override_id = 0
manual_override_key = ''
# 缓存无人机状态（后台线程更新，API 只读）
_cached_drone_state = {
    'x': 0, 'y': 0, 'z': 0,
    'wp_index': 0, 'wp_total': 0,
    'ok': False, 'error': '',
}
_state_poller_thread = None
_state_poller_running = False
# 自动附着视线：仿真器如果在后端启动之后才起来，自动连上去，用户不用再回界面点"连接"。
_carla_watchdog_thread = None
_carla_watchdog_running = True
# 用户主动点"停止"后的一段时间内不做自动重连，避免把正在关闭的仿真器又接上。
_carla_watchdog_paused_until = 0.0

carla_lock = threading.Lock()
carla_ready = False
current_map_name = None
MAP_OFFSETS_PATH = Path(__file__).with_name('map_offsets.json')
MAP_TOPOLOGY_CACHE_DIR = Path(__file__).with_name('recordings') / 'map_topology'
MAP_TOPOLOGY_CACHE_VERSION = 1
# 预览坐标对齐只在后端进程内缓存，不修改已经验证过的固定航线文件。
ROUTE_ALIGNMENT_CACHE = {}
BACKEND_VERSION = '7.7-map-alignment-v2'

if carla_air_dir:
    CARLA_START_SCRIPT = os.path.join(carla_air_dir, 'StartCarlaAir.bat')
    CARLA_STOP_SCRIPT = os.path.join(carla_air_dir, 'StopCarlaAir.bat')
else:
    CARLA_START_SCRIPT = None
    CARLA_STOP_SCRIPT = None

def find_drone_actor(world):
    for actor in world.get_actors():
        if 'drone' in actor.type_id:
            return actor
    return None

def connect_carla():
    global client, world, carla_ready, current_map_name, drone_actor
    if carla is None:
        return False
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(15.0)
        world = client.get_world()
        world.wait_for_tick(5.0)
        carla_ready = True
        print("已连接到 CARLA-Air，世界稳定")
        current_map_name = world.get_map().name
        drone_actor = find_drone_actor(world)

        if drone_actor:
            print(f"找到无人机: {drone_actor.type_id} (id={drone_actor.id})")
        else:
            print("警告：未找到无人机 actor，请检查AirSim的settings.json内是否设置为  AutoCreate: true ")

        return True
    except Exception as e:
        print(f"CARLA-Air连接失败: {e}")
        carla_ready = False
        current_map_name = None
        drone_actor = None
        return False

'''同步模式'''
def enable_synchronous_mode(fps):
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / fps
    world.apply_settings(settings)

'''异步模式'''
def disable_synchronous_mode():
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0.0
    world.apply_settings(settings)

'''查询CarlaAir状态'''
@app.route('/api/status', methods=['GET'])
def get_status():
    return jsonify({
        'connected': carla_ready,
        'map': current_map_name,
        'drone_found': drone_actor is not None
    })


@app.route('/api/version', methods=['GET'])
def get_backend_version():
    """供 Electron 启动握手使用，避免复用旧版本后台进程。"""
    return jsonify({'success': True, 'version': BACKEND_VERSION})


@app.route('/api/weather/set', methods=['POST'])
def set_weather():
    with carla_lock:
        if not carla_ready:
            return jsonify({'success': False, 'error': '未连接到 CARLA'}), 503
        try:
            data = request.json
            params = data.get('params', {})
            weather = carla.WeatherParameters(
                cloudiness=params.get('cloudiness', 0),
                precipitation=params.get('precipitation', 0),
                precipitation_deposits=params.get('precipitation_deposits', 0),
                wind_intensity=params.get('wind_intensity', 0),
                fog_density=params.get('fog_density', 0),
                fog_distance=params.get('fog_distance', 0),
                fog_falloff=params.get('fog_falloff', 0.0),
                wetness=params.get('wetness', 0),
                sun_azimuth_angle=params.get('sun_azimuth_angle', 0),
                sun_altitude_angle=params.get('sun_altitude_angle', 0)
            )
            world.set_weather(weather)
            return jsonify({'success': True})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/sensors/spawn', methods=['POST'])
def spawn_sensors():
    global active_sensors, sensor_save_dir, drone_actor
    with carla_lock:
        if not carla_ready or world is None:
            return jsonify({'success': False, 'error': '未连接到 CARLA'}), 503
        if drone_actor is None:
            drone_actor = find_drone_actor(world)
        if drone_actor is None:
            return jsonify({'success': False, 'error': '未找到无人机'}), 500

        try:
            data = request.json
            configs = data.get('sensorConfigs', {})
            save_dir = data.get('saveDir', './datasets/default')
            sensor_save_dir = save_dir

            print(f"[Spawn] 收到传感器配置: {list(configs.keys())}")

            '''销毁上次残余的传感器，如果有'''
            destroy_sensors(active_sensors)

            def spawn_with_retry(spawn_func, config, sensor_id, retries=3):
                for attempt in range(retries):
                    global drone_actor
                    if drone_actor is None or not drone_actor.is_alive:
                        drone_actor = find_drone_actor(world)
                        if drone_actor is None:
                            print(f"[Spawn] 无人机未找到，等待 0.5 秒后重试...")
                            time.sleep(0.5)
                            continue
                    try:
                        sensor = spawn_func(world, drone_actor, config)
                        return sensor
                    except Exception as e:
                        print(f"[Spawn] {sensor_id} 尝试 {attempt+1}/{retries} 失败: {e}")
                        time.sleep(0.5)
                raise Exception(f"传感器 {sensor_id} 生成失败，重试 {retries} 次后仍失败")

            for sensor_id, config in configs.items():
                if not config.get('enabled'):
                    print(f"[Spawn] 传感器 {sensor_id} 未启用，跳过")
                    continue
                if config.get('source') == 'airsim':
                    print(f"[Spawn] 跳过 AirSim 传感器 {sensor_id}")
                    continue

                try:
                    if sensor_id == 'camera_rgb':
                        sensor = spawn_with_retry(spawn_rgb_camera, config, sensor_id)
                    elif sensor_id == 'camera_depth':
                        sensor = spawn_with_retry(spawn_depth_camera, config, sensor_id)
                    elif sensor_id == 'camera_semantic':
                        sensor = spawn_with_retry(spawn_semantic_camera, config, sensor_id)
                    elif sensor_id == 'camera_instance':
                        sensor = spawn_with_retry(spawn_instance_camera, config, sensor_id)
                    elif sensor_id == 'camera_optical_flow':
                        sensor = spawn_with_retry(spawn_optical_flow_camera, config, sensor_id)
                    elif sensor_id == 'camera_normals':
                        sensor = spawn_with_retry(spawn_normals_camera, config, sensor_id)
                    elif sensor_id == 'lidar':
                        sensor = spawn_with_retry(spawn_lidar, config, sensor_id)
                    elif sensor_id == 'semantic_lidar':
                        sensor = spawn_with_retry(spawn_semantic_lidar, config, sensor_id)
                    elif sensor_id == 'radar':
                        sensor = spawn_with_retry(spawn_radar, config, sensor_id)
                    elif sensor_id == 'gnss':
                        sensor = spawn_with_retry(spawn_gnss, config, sensor_id)
                    elif sensor_id == 'imu':
                        sensor = spawn_with_retry(spawn_imu, config, sensor_id)
                    else:
                        print(f"[Spawn] 未实现的 CARLA 传感器: {sensor_id}")
                        continue

                    active_sensors.append((sensor, sensor_id))
                    print(f"[Spawn] 生成成功: {sensor_id} (ID={sensor.id})")
                except Exception as e:
                    print(f"[Spawn] 传感器 {sensor_id} 最终失败，跳过: {e}")

            print(f"[Spawn] 成功生成 {len(active_sensors)} 个 CARLA 传感器")
            return jsonify({'success': True, 'spawned': len(active_sensors)})
        except Exception as e:
            print(f"[Spawn] 异常: {e}")
            import traceback
            traceback.print_exc()
            return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/sensors/stop', methods=['POST'])
def stop_sensors():
    global active_sensors
    with carla_lock:
        destroy_sensors(active_sensors)
    return jsonify({'success': True})


@app.route('/api/weather/sequence', methods=['POST'])
def generate_sequence():
    global active_sensors, sensor_save_dir, airsim_client, airsim_connected
    global cruise_controller, cruise_running, _state_poller_thread, _state_poller_running
    global world, drone_actor, cruise_state
    with carla_lock:
        if not carla_ready:
            return jsonify({'success': False, 'error': '未连接到 CARLA'}), 503
        try:
            data = request.json
            frames = data.get('frames', [])
            fps = data.get('fps', 10)
            sensors_full_config = data.get('sensorsFullConfig', {})
            cruise_speed = float(data.get('cruiseSpeed', 8))
            cruise_altitude = float(data.get('cruiseAltitude', 45))

            if not frames:
                return jsonify({'success': False, 'error': '帧数据为空'}), 400

            # 采集只读取传感器；飞控始终由唯一的巡航线程驱动。
            if not cruise_running:
                print("[Sequence] 自动启动巡航...")
                try:
                    _start_road_cruise(
                        cruise_speed,
                        cruise_altitude,
                        data.get('route_save_dir', './recordings/drone_routes'),
                        data.get('route_id'),
                    )
                except Exception as e:
                    # 启动校验失败时受控降落，不能直接释放 API 造成失控下落。
                    _land_drone()
                    return jsonify({'success': False, 'error': f'巡航启动失败: {e}'}), 500

            total_frames = len(frames)
            first_params = frames[0].get('weatherParams', {})
            is_static = all(
                frame.get('weatherParams', {}) == first_params
                for frame in frames
            )

            enable_synchronous_mode(fps)

            if active_sensors:
                start_listeners(active_sensors, sensor_save_dir, total_frames)

            airsim_enabled = []
            if airsim_connected and airsim_client is not None:
                for sensor_id, cfg in sensors_full_config.items():
                    if cfg.get('enabled') and cfg.get('source') == 'airsim':
                        airsim_enabled.append(sensor_id)
                if airsim_enabled:
                    print(f"[AirSim] 将采集 {len(airsim_enabled)} 个传感器: {airsim_enabled}")

            frame_duration = 1.0 / fps

            for frame_idx in range(total_frames):
                start_time = time.time()

                params = frames[frame_idx].get('weatherParams', {})
                weather = carla.WeatherParameters(
                    cloudiness=params.get('cloudiness', 0),
                    precipitation=params.get('precipitation', 0),
                    precipitation_deposits=params.get('precipitation_deposits', 0),
                    wind_intensity=params.get('wind_intensity', 0),
                    fog_density=params.get('fog_density', 0),
                    fog_distance=params.get('fog_distance', 0),
                    fog_falloff=params.get('fog_falloff', 0.0),
                    wetness=params.get('wetness', 0),
                    sun_azimuth_angle=params.get('sun_azimuth_angle', 0),
                    sun_altitude_angle=params.get('sun_altitude_angle', 0)
                )
                world.set_weather(weather)

                world.tick()

                if cruise_state == 'manual_override':
                    raise RuntimeError('检测到飞行控制按键，自动巡航已退出，控制权已交还给用户')

                if airsim_connected and airsim_enabled:
                    try:
                        collect_airsim_frame(
                            airsim_client, sensors_full_config, airsim_enabled,
                            sensor_save_dir, frame_idx
                        )
                    except Exception as e:
                        print(f"[AirSim] 第 {frame_idx} 帧采集失败: {e}")

                elapsed = time.time() - start_time
                if elapsed < frame_duration:
                    time.sleep(frame_duration - elapsed)

                if (frame_idx + 1) % 10 == 0:
                    print(f"进度: {frame_idx + 1}/{total_frames} 帧")

            # 采集结束后先停止飞控，再把操作权交还给用户。
            destroy_sensors(active_sensors)
            disable_synchronous_mode()
            # 正常采集完成后自动降落；下一次生成数据集会重新起飞到设定高度。
            _land_drone()

            return jsonify({'success': True, 'total_frames': total_frames})

        except Exception as e:
            try:
                # 采集异常也必须走完整降落流程；只释放 API 控制权会让无人机停在空中。
                _land_drone()
            except Exception as land_error:
                print(f"[Sequence] 异常后的自动降落失败: {land_error}")
            try:
                disable_synchronous_mode()
                destroy_sensors(active_sensors)
            except:
                pass
            print(f"[Sequence] 异常: {e}")
            import traceback
            traceback.print_exc()
            return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/carla/connect', methods=['POST'])
def connect_existing_carla():
    """连接已经在运行的 Carla-Air，不重新启动仿真器。

    适用场景：用户自己在外面双击了 StartCarlaAir.bat，或者仿真器还开着，
    此接口只做"附着"，避免重复启动导致把已经跑起来的仿真器杀掉重开。
    """
    global _state_poller_thread, _state_poller_running
    global airsim_client, airsim_connected
    with carla_lock:
        if carla is None:
            return jsonify({'success': False, 'error': 'carla 模块未加载，请检查 Python 环境'}), 500
        try:
            if not connect_carla():
                return jsonify({
                    'success': False,
                    'error': '连接失败：CARLA 端口 2000 无响应。请确认 Carla-Air 已启动。',
                }), 503
            # 连接成功后启动无人机状态轮询，让界面能显示位置/巡航状态。
            if not _state_poller_running and airsim is not None:
                try:
                    airsim_client = airsim.MultirotorClient(port=41451)
                    airsim_client.confirmConnection()
                    airsim_connected = True
                    _state_poller_running = True
                    _state_poller_thread = threading.Thread(target=_state_poller, daemon=True)
                    _state_poller_thread.start()
                except Exception as airsim_error:
                    airsim_connected = False
                    print(f"[Connect] AirSim 未就绪（不影响 CARLA 侧功能）: {airsim_error}")
            return jsonify({
                'success': True,
                'map': current_map_name,
                'drone_found': drone_actor is not None,
                'airsim_connected': airsim_connected,
                'message': f'已连接 {current_map_name}',
            })
        except Exception as e:
            print(f"[Connect] 连接已有 Carla-Air 失败: {e}")
            return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/carla/launch', methods=['POST'])
def launch_carla():
    global client, world, carla_ready, current_map_name, drone_actor, active_sensors
    global airsim_client, airsim_connected
    with carla_lock:
        try:
            data = request.get_json()
            quality = data.get('quality', 'Epic')
            map_name = data.get('map', 'Town10HD')
            airsim_params = data.get('airsimParams', {})

            if airsim_params:
                settings_path = airsim_params.get('settingsPath')
                if not settings_path:
                    carla_dir = get_carla_air_dir()
                    settings_path = os.path.join(carla_dir, 'settings.json')
                sensors_dict = airsim_params.get('sensors', {})
                generate_airsim_settings(settings_path, airsim_params, sensors_dict)
                print(f"[AirSim] settings.json 已更新: {settings_path}")
            else:
                print("[AirSim] 无 AirSim 配置，跳过 settings.json 生成")

            if not CARLA_START_SCRIPT:
                return jsonify({'success': False, 'error': 'Carla-Air 路径未配置'})
            print(f"启动 Carla-Air，地图: {map_name}，画质: {quality}")
            if not os.path.exists(CARLA_START_SCRIPT):
                return jsonify({'success': False, 'error': f'启动脚本不存在: {CARLA_START_SCRIPT}'})

            subprocess.Popen([CARLA_START_SCRIPT, map_name, '--quality', quality],
                             shell=True, creationflags=subprocess.CREATE_NO_WINDOW)

            print("等待 Carla-Air 启动...")
            for i in range(60):
                time.sleep(1)
                if connect_carla():
                    print("Carla 连接成功！")
                    break
            else:
                return jsonify({'success': False, 'error': 'Carla-Air 启动超时（60秒）'})

            if airsim is not None:
                try:
                    airsim_client = airsim.MultirotorClient()
                    airsim_client.confirmConnection()
                    airsim_client.enableApiControl(True)
                    airsim_client.armDisarm(True)
                    airsim_connected = True
                    print("AirSim 连接成功")
                except Exception as e:
                    print(f"AirSim 连接失败: {e}")
                    airsim_connected = False
            else:
                airsim_connected = False

            return jsonify({'success': True})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/carla/stop', methods=['POST'])
def stop_carla():
    global  client, world, carla_ready, current_map_name, drone_actor, active_sensors
    global airsim_connected, airsim_client
    global _state_poller_running, _state_poller_thread
    global _carla_watchdog_paused_until
    with carla_lock:
        try:
            # 用户主动停止时，短时间内不要再自动附着（仿真器正在关闭，端口可能还没释放）。
            _carla_watchdog_paused_until = time.time() + 20
            # 关闭仿真器前先退出飞控，避免后台线程继续向失效连接发送命令。
            _stop_cruise(release_control=False)
            _state_poller_running = False
            if _state_poller_thread is not None:
                _state_poller_thread.join(timeout=2)
                _state_poller_thread = None
            destroy_sensors(active_sensors)
            if CARLA_STOP_SCRIPT and os.path.exists(CARLA_STOP_SCRIPT):
                subprocess.Popen(CARLA_STOP_SCRIPT, shell=True, creationflags=subprocess.CREATE_NO_WINDOW)
            else:
                subprocess.Popen('taskkill /f /im CarlaUE4.exe', shell=True)
            carla_ready = False
            current_map_name = None
            client = None
            world = None
            drone_actor = None
            airsim_connected = False
            airsim_client = None
            return jsonify({'success': True})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500


def _short_map_name(name):
    """将 CARLA 的完整地图路径统一转换为 TownXX 短名称。"""
    return str(name or '').replace('\\', '/').split('/')[-1]


def _topology_cache_path(map_name):
    """返回固定二维地图缓存路径，并拒绝路径穿越字符。"""
    short_name = _short_map_name(map_name)
    if not re.fullmatch(r'[A-Za-z0-9_]+', short_name):
        return None
    return MAP_TOPOLOGY_CACHE_DIR / f'{short_name}_topology.json'


def _load_topology_cache(map_name):
    cache_path = _topology_cache_path(map_name)
    if cache_path is None or not cache_path.is_file():
        return None
    try:
        data = json.loads(cache_path.read_text(encoding='utf-8'))
        if data.get('cache_version') != MAP_TOPOLOGY_CACHE_VERSION:
            return None
        data.pop('cache_version', None)
        data['source'] = 'fixed_cache'
        return data
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return None


def _save_topology_cache(topology):
    cache_path = _topology_cache_path(topology.get('map'))
    if cache_path is None:
        return
    try:
        MAP_TOPOLOGY_CACHE_DIR.mkdir(parents=True, exist_ok=True)
        payload = dict(topology)
        payload['cache_version'] = MAP_TOPOLOGY_CACHE_VERSION
        cache_path.write_text(json.dumps(payload, ensure_ascii=False), encoding='utf-8')
    except OSError as exc:
        # 缓存写入失败不影响本次预览，下一次仍可从 CARLA 重新生成。
        print(f'[Map] 固定二维地图缓存写入失败: {exc}')


def _read_map_topology(target_map, sample_distance=3.0):
    """读取道路采样点，整理成前端可以直接绘制的二维车道线。"""
    sample_distance = max(1.5, min(float(sample_distance), 10.0))
    lanes = {}
    topology_edges = target_map.get_topology()

    def add_waypoint(waypoint):
        location = waypoint.transform.location
        road_id = int(getattr(waypoint, 'road_id', -1))
        section_id = int(getattr(waypoint, 'section_id', -1))
        lane_id = int(getattr(waypoint, 'lane_id', 0))
        lane_type = str(getattr(waypoint, 'lane_type', 'Unknown')).split('.')[-1]
        key = (road_id, section_id, lane_id, lane_type)
        point = [
            round(float(location.x), 3),
            round(float(location.y), 3),
            round(float(location.z), 3),
        ]
        signature = tuple(round(value, 2) for value in point)
        lane = lanes.setdefault(key, {'points': [], 'seen': set()})
        if signature in lane['seen']:
            return
        lane['seen'].add(signature)
        lane['points'].append((float(getattr(waypoint, 's', 0.0)), point))

    # 拓扑端点保证短路段也能显示，采样点负责补齐弯道细节。
    for start_waypoint, end_waypoint in topology_edges:
        add_waypoint(start_waypoint)
        add_waypoint(end_waypoint)
    for waypoint in target_map.generate_waypoints(sample_distance):
        add_waypoint(waypoint)

    roads = []
    for (road_id, section_id, lane_id, lane_type), lane in lanes.items():
        lane['points'].sort(key=lambda item: item[0])
        points = [point for _, point in lane['points']]
        if len(points) < 2:
            continue
        roads.append({
            'id': f'{road_id}:{section_id}:{lane_id}',
            'road_id': road_id,
            'section_id': section_id,
            'lane_id': lane_id,
            'lane_type': lane_type,
            'points': points,
        })

    all_points = [point for road in roads for point in road['points']]
    if not all_points:
        raise RuntimeError('当前地图没有可绘制的道路航点')

    xs = [point[0] for point in all_points]
    ys = [point[1] for point in all_points]
    return {
        'map': _short_map_name(target_map.name),
        'projection': 'carla_xy',
        'sample_distance': sample_distance,
        'topology_edges': len(topology_edges),
        'road_count': len(set(road['road_id'] for road in roads)),
        'lane_count': len(roads),
        'roads': roads,
        'bounds': {
            'min_x': round(min(xs), 3),
            'max_x': round(max(xs), 3),
            'min_y': round(min(ys), 3),
            'max_y': round(max(ys), 3),
        },
    }


@app.route('/api/map/topology', methods=['GET'])
def get_map_topology():
    """返回当前 CARLA 地图的二维道路拓扑数据。"""
    with carla_lock:
        try:
            requested_map = _short_map_name(request.args.get('map', '').strip())
            cached_topology = _load_topology_cache(requested_map) if requested_map else None
            if cached_topology is not None:
                return jsonify({'success': True, **cached_topology})

            if not carla_ready or world is None:
                return jsonify({'success': False, 'error': '未连接到 CARLA，且当前地图尚未生成固定缓存'}), 503

            current_map = _short_map_name(world.get_map().name)
            if requested_map and requested_map != current_map:
                return jsonify({
                    'success': False,
                    'error': f'当前 CARLA 地图是 {current_map}，请先加载 {requested_map}',
                    'current_map': current_map,
                }), 409

            try:
                sample_distance = float(request.args.get('sample_distance', '3.0'))
            except (TypeError, ValueError):
                sample_distance = 3.0
            topology = _read_map_topology(world.get_map(), sample_distance)
            _save_topology_cache(topology)
            return jsonify({'success': True, 'source': 'carla_live', **topology})
        except Exception as exc:
            print(f'[Map] 道路拓扑读取失败: {exc}')
            return jsonify({'success': False, 'error': str(exc)}), 500


@app.route('/api/map/fixed-route', methods=['GET'])
def get_fixed_route_preview():
    """读取已验证的固定巡航航线，并转换为地图预览使用的 CARLA XY 坐标。"""
    try:
        requested_map = _short_map_name(request.args.get('map', '').strip())
        if not requested_map:
            return jsonify({'success': False, 'error': '未指定地图'}), 400
        if load_fixed_cruise_route is None:
            return jsonify({'success': False, 'error': '固定航线模块未加载'}), 503

        route = load_fixed_cruise_route(requested_map)
        alignment = _get_route_alignment(requested_map, route.waypoints)
        offset_x, offset_y = alignment['offset']
        waypoints = [
            {
                # 固定航线保存的是 AirSim XY，减去 AirSim-CARLA 偏移后落到 CARLA XY。
                'x': round(float(point.x) - offset_x, 3),
                'y': round(float(point.y) - offset_y, 3),
                'z': round(float(point.z), 3),
                'yaw': None if point.yaw is None else round(float(point.yaw), 3),
            }
            for point in route.waypoints
        ]
        return jsonify({
            'success': True,
            'map': normalize_map_name(route.map_name),
            'source': route.source,
            'source_frame': 'airsim_ned_xy',
            'display_frame': 'carla_xy',
            'alignment': alignment['mode'],
            'offset': {'x': round(offset_x, 3), 'y': round(offset_y, 3)},
            'alignment_error': alignment['error'],
            'alignment_max_error': alignment['max_error'],
            'alignment_source': alignment['source'],
            'waypoints': waypoints,
            'waypoint_count': len(waypoints),
        })
    except Exception as exc:
        print(f'[Map] 固定巡航路线读取失败: {exc}')
        return jsonify({'success': False, 'error': str(exc)}), 404


@app.route('/api/route/plan', methods=['POST'])
def plan_custom_route():
    """把用户点击的道路点规划成沿道路连通的 CARLA 折线（预览用）。"""
    with carla_lock:
        if not carla_ready or world is None:
            return jsonify({'success': False, 'error': '未连接到 CARLA，无法规划自定义航线'}), 503
        if plan_route_carla is None:
            return jsonify({'success': False, 'error': '自定义航线模块未加载'}), 503
        try:
            data = request.get_json(silent=True) or {}
            points = data.get('points', [])
            if not isinstance(points, list) or len(points) < 2:
                return jsonify({'success': False, 'error': '至少需要起点和终点两个点'}), 400
            polyline, error, warnings = plan_route_carla(world.get_map(), points)
            if polyline is None:
                return jsonify({'success': False, 'error': error, 'warnings': warnings}), 422
            distance = sum(
                math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(polyline, polyline[1:])
            )
            return jsonify({
                'success': True,
                'map': _short_map_name(world.get_map().name),
                'node_count': len(polyline),
                'total_distance': round(distance, 2),
                'polyline': [
                    {'x': round(p[0], 3), 'y': round(p[1], 3), 'z': round(p[2], 3)}
                    for p in polyline
                ],
                'warnings': warnings,
            })
        except Exception as exc:
            print(f'[Route] 自定义航线规划失败: {exc}')
            return jsonify({'success': False, 'error': str(exc)}), 500


@app.route('/api/route/save', methods=['POST'])
def save_custom_route_api():
    """规划并把自定义航线保存为可执行的 AirSim 航线 JSON。"""
    with carla_lock:
        if not carla_ready or world is None:
            return jsonify({'success': False, 'error': '未连接到 CARLA'}), 503
        if plan_route_carla is None or carla_polyline_to_airsim_route is None or save_custom_route is None:
            return jsonify({'success': False, 'error': '自定义航线模块未加载'}), 503
        try:
            data = request.get_json(silent=True) or {}
            points = data.get('points', [])
            altitude = float(data.get('altitude', 45))
            if not isinstance(points, list) or len(points) < 2:
                return jsonify({'success': False, 'error': '至少需要起点和终点两个点'}), 400
            map_name = _short_map_name(world.get_map().name)
            polyline, error, warnings = plan_route_carla(world.get_map(), points)
            if polyline is None:
                return jsonify({'success': False, 'error': error, 'warnings': warnings}), 422
            offset = _resolve_custom_route_offset(map_name)
            reference_ground_z = polyline[0][2] if polyline else 0.0
            route = carla_polyline_to_airsim_route(
                map_name, polyline, altitude, offset, reference_ground_z,
            )
            route_id = str(int(time.time()))
            saved_path = save_custom_route(route, route_id)
            return jsonify({
                'success': True,
                'route_id': route_id,
                'path': str(saved_path),
                'map': map_name,
                'waypoint_count': len(route.waypoints),
                'alignment_offset': {'x': round(offset[0], 3), 'y': round(offset[1], 3)},
                'warnings': warnings,
            })
        except Exception as exc:
            print(f'[Route] 自定义航线保存失败: {exc}')
            return jsonify({'success': False, 'error': str(exc)}), 500


@app.route('/api/route/custom', methods=['GET'])
def list_custom_routes_api():
    """列出某张地图已保存的自定义航线 id。"""
    try:
        if list_custom_routes is None:
            return jsonify({'success': False, 'error': '自定义航线模块未加载'}), 503
        map_name = _short_map_name(request.args.get('map', '').strip())
        if not map_name:
            return jsonify({'success': False, 'error': '未指定地图'}), 400
        return jsonify({'success': True, 'map': map_name, 'route_ids': list_custom_routes(map_name)})
    except Exception as exc:
        return jsonify({'success': False, 'error': str(exc)}), 500


@app.route('/api/map/load', methods=['POST'])
def load_map():
    global world, client, carla_ready, current_map_name, drone_actor
    global airsim_client, airsim_connected
    global _state_poller_thread, _state_poller_running, cruise_altitude, cruise_state
    with carla_lock:
        if not carla_ready or client is None:
            return jsonify({'success': False, 'error': '未连接到 CARLA'}), 503
        try:
            data = request.json
            map_name = data.get('map', 'Town10HD')
            print(f"准备切换地图到: {map_name}")

            # 已在目标地图上时直接返回成功，避免无谓的世界重载（会降落无人机并重置 AirSim 连接）。
            current = world.get_map().name if world is not None else ''
            if _short_map_name(current) == _short_map_name(map_name):
                print(f"已在目标地图 {_short_map_name(current)}，跳过重载")
                return jsonify({
                    'success': True,
                    'skipped': True,
                    'map': current,
                    'message': f'已在地图 {_short_map_name(current)}',
                })

            # 地图切换采用“落地后重启”的生命周期，不能让旧地图的空中任务带入新地图。
            try:
                _land_drone()
            except Exception as land_error:
                # 即使旧地图的降落 RPC 不响应，也继续卸载地图，防止界面被旧任务永久卡住。
                print(f"[Map] 切换前自动降落失败，继续切图: {land_error}")

            client.set_timeout(30.0)
            world = client.load_world(map_name)
            world.wait_for_tick(5.0)
            carla_ready = True
            current_map_name = world.get_map().name
            drone_actor = _wait_for_drone_actor(world)
            if drone_actor is None:
                raise RuntimeError('地图加载后未在 15 秒内找到无人机 actor')

            # AirSim 服务会随地图重置，不能复用旧 client 或旧飞行任务。
            airsim_client = None
            airsim_connected = False
            cruise_state = 'manual'

            print(f"地图切换成功: {current_map_name}")
            return jsonify({'success': True, 'map': current_map_name, 'airborne_restored': False})
        except Exception as e:
            print(f"地图切换异常: {e}")
            return jsonify({'success': False, 'error': str(e)}), 500


def _map_offset_key(map_name):
    """将地图名称统一为 map_offsets.json 使用的键。"""
    return _short_map_name(map_name)


def _configured_map_offset(map_name):
    """Read a verified CARLA-to-AirSim XY offset when this map has one."""
    try:
        data = json.loads(MAP_OFFSETS_PATH.read_text(encoding='utf-8'))
        offset = data.get(_map_offset_key(map_name))
        if offset is not None:
            return float(offset['x']), float(offset['y'])
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def _topology_xy_points(map_name):
    """读取缓存道路的 XY 点，供预览航线做平移匹配。"""
    topology = _load_topology_cache(map_name)
    if not topology:
        return []
    return [
        (float(point[0]), float(point[1]))
        for road in topology.get('roads', [])
        for point in road.get('points', [])
        if len(point) >= 2
    ]


def _build_xy_grid(points, cell_size=5.0):
    """建立简单空间网格，避免每个航点都遍历整张道路地图。"""
    grid = {}
    for x_value, y_value in points:
        key = (math.floor(x_value / cell_size), math.floor(y_value / cell_size))
        grid.setdefault(key, []).append((x_value, y_value))
    bounds = (
        min(point[0] for point in points),
        max(point[0] for point in points),
        min(point[1] for point in points),
        max(point[1] for point in points),
    )
    return grid, bounds


def _nearest_topology_distance(grid, bounds, x_value, y_value, cell_size=5.0):
    """返回点到附近道路采样点的距离；超出网格时使用边界距离惩罚。"""
    cell_x = math.floor(x_value / cell_size)
    cell_y = math.floor(y_value / cell_size)
    nearest_squared = None
    # 道路采样间距约为 3m，检查 5x5 个网格即可覆盖相邻道路采样点。
    for offset_x in range(-2, 3):
        for offset_y in range(-2, 3):
            for road_x, road_y in grid.get((cell_x + offset_x, cell_y + offset_y), ()):
                distance_squared = (x_value - road_x) ** 2 + (y_value - road_y) ** 2
                if nearest_squared is None or distance_squared < nearest_squared:
                    nearest_squared = distance_squared
    if nearest_squared is not None:
        return math.sqrt(nearest_squared)

    # 固定航线起点可能包含从出生点到第一条道路的连接段，不能让单个离群点否决整条航线。
    distance_x = max(bounds[0] - x_value, 0.0, x_value - bounds[1])
    distance_y = max(bounds[2] - y_value, 0.0, y_value - bounds[3])
    return min(200.0, math.hypot(distance_x, distance_y) + 20.0)


def _route_alignment_score(route_xy, grid, bounds, offset):
    """计算某个 AirSim-CARLA 平移量的道路贴合误差。"""
    if not route_xy:
        return (float('inf'), float('inf'), float('inf'))
    sample_count = min(40, len(route_xy))
    sample_indices = {
        round(index * (len(route_xy) - 1) / max(1, sample_count - 1))
        for index in range(sample_count)
    }
    distances = [
        _nearest_topology_distance(
            grid,
            bounds,
            route_xy[index][0] - offset[0],
            route_xy[index][1] - offset[1],
        )
        for index in sorted(sample_indices)
    ]
    distances.sort()
    # 少量起飞连接段可能不在道路上，使用 90% 核心样本并保留 P90 约束。
    core_count = max(8, int(len(distances) * 0.9))
    core_mean = sum(distances[:core_count]) / core_count
    p90 = distances[min(len(distances) - 1, max(0, int(len(distances) * 0.9) - 1))]
    return (core_mean + 0.25 * p90, p90, distances[-1])


def _derive_route_alignment(map_name, route_waypoints, topology_points=None):
    """从道路缓存反推固定航线的 AirSim-CARLA XY 平移量。"""
    route_xy = [
        (float(point.x), float(point.y))
        for point in route_waypoints
    ]
    topology_points = topology_points or _topology_xy_points(map_name)
    if len(route_xy) < 2 or len(topology_points) < 2:
        return None

    # 限制候选点数量，首次打开地图时也能在秒级完成匹配。
    stride = max(1, len(topology_points) // 1800)
    candidate_points = topology_points[::stride]
    grid, bounds = _build_xy_grid(topology_points)
    anchor_indices = [
        0,
        len(route_xy) // 4,
        len(route_xy) // 2,
        (len(route_xy) * 3) // 4,
        len(route_xy) - 1,
    ]
    candidates = set()
    for anchor_index in anchor_indices:
        anchor_x, anchor_y = route_xy[anchor_index]
        for road_x, road_y in candidate_points:
            candidates.add((round(anchor_x - road_x, 1), round(anchor_y - road_y, 1)))

    best_offset = None
    best_score = (float('inf'), float('inf'), float('inf'))
    for offset in candidates:
        score = _route_alignment_score(route_xy, grid, bounds, offset)
        if score < best_score:
            best_offset, best_score = offset, score

    if best_offset is None:
        return None

    # 在粗略候选附近用 0.5m 步长细化，减少道路采样间距带来的残余偏差。
    refined_offset = best_offset
    refined_score = best_score
    for delta_x in range(-6, 7):
        for delta_y in range(-6, 7):
            offset = (best_offset[0] + delta_x * 0.5, best_offset[1] + delta_y * 0.5)
            score = _route_alignment_score(route_xy, grid, bounds, offset)
            if score < refined_score:
                refined_offset, refined_score = offset, score
    return {
        'offset': refined_offset,
        'score': refined_score,
    }


def _get_route_alignment(map_name, route_waypoints):
    """优先使用人工校准值，否则自动将 AirSim 航线匹配到 CARLA 道路。"""
    cache_key = _short_map_name(map_name)
    cached = ROUTE_ALIGNMENT_CACHE.get(cache_key)
    if cached is not None:
        return cached

    configured_offset = _configured_map_offset(map_name)
    topology_points = _topology_xy_points(map_name)
    derived = _derive_route_alignment(map_name, route_waypoints, topology_points) if topology_points else None

    candidates = []
    if configured_offset is not None and topology_points:
        grid, bounds = _build_xy_grid(topology_points)
        configured_score = _route_alignment_score(
            [(float(point.x), float(point.y)) for point in route_waypoints],
            grid,
            bounds,
            configured_offset,
        )
        candidates.append((configured_score, configured_offset, 'configured_offset', 'map_offsets.json'))
    if derived is not None:
        candidates.append((derived['score'], derived['offset'], 'auto_matched', 'topology_cache'))

    if candidates:
        score, offset, mode, source = min(candidates, key=lambda item: item[0])
        # 误差过大时不伪造“已对齐”，保留原点坐标并让前端显示警告。
        if score[0] <= 12.0:
            result = {
                'offset': (float(offset[0]), float(offset[1])),
                'mode': mode,
                'source': source,
                'error': round(float(score[0]), 3),
                'max_error': round(float(score[2]), 3),
            }
        else:
            result = {
                'offset': (0.0, 0.0),
                'mode': 'identity_fallback',
                'source': 'alignment_failed',
                'error': round(float(score[0]), 3),
                'max_error': round(float(score[2]), 3),
            }
    elif configured_offset is not None:
        result = {
            'offset': (float(configured_offset[0]), float(configured_offset[1])),
            'mode': 'configured_offset',
            'source': 'map_offsets.json',
            'error': None,
            'max_error': None,
        }
    else:
        result = {
            'offset': (0.0, 0.0),
            'mode': 'identity',
            'source': 'no_topology_cache',
            'error': None,
            'max_error': None,
        }

    ROUTE_ALIGNMENT_CACHE[cache_key] = result
    return result


def _estimate_airsim_offset(ac, drone_actor, map_name=''):
    """估算 CARLA → AirSim NED 坐标平移偏移 (AirSim_pos - CARLA_pos)"""
    try:
        state = ac.getMultirotorState()
        airsim_pos = state.kinematics_estimated.position
        carla_loc = drone_actor.get_transform().location
        # 注意: 原始代码是 AirSim - CARLA，不是 CARLA - AirSim。
        live_offset = (float(airsim_pos.x_val) - float(carla_loc.x),
                       float(airsim_pos.y_val) - float(carla_loc.y))
        configured_offset = _configured_map_offset(map_name)
        if configured_offset is not None:
            mismatch = ((configured_offset[0] - live_offset[0]) ** 2
                        + (configured_offset[1] - live_offset[1]) ** 2) ** 0.5
            # 已标定偏移只在与实时 actor 配对结果一致时使用，避免旧标定把路线整体移错。
            if mismatch <= 5.0:
                print(f"[Cruise] 使用 { _map_offset_key(map_name) } 已标定坐标偏移")
                return configured_offset
            print(f"[Cruise] 已标定偏移与实时位置相差 {mismatch:.1f}m，使用实时偏移")
        return live_offset
    except Exception:
        return (0.0, 0.0)


def _resolve_custom_route_offset(map_name):
    """为自定义航线确定 CARLA->AirSim 的 XY 偏移：实时配对准，回退到固定航线对齐。"""
    if airsim_connected and airsim_client is not None and drone_actor is not None:
        offset = _estimate_airsim_offset(airsim_client, drone_actor, map_name)
        if offset != (0.0, 0.0):
            return offset
    try:
        if load_fixed_cruise_route is not None:
            fixed = load_fixed_cruise_route(map_name)
            alignment = _get_route_alignment(map_name, fixed.waypoints)
            return (float(alignment['offset'][0]), float(alignment['offset'][1]))
    except Exception:
        pass
    return (0.0, 0.0)


def _road_ground_z(target_world, location):
    """Return the CARLA road elevation below the current drone position."""
    try:
        waypoint = target_world.get_map().get_waypoint(location, project_to_road=True)
        return float(waypoint.transform.location.z) if waypoint is not None else 0.0
    except Exception:
        return 0.0


def _wait_for_drone_actor(target_world, timeout=15.0):
    """等待地图切换后 AirSim 创建新的 CARLA 无人机 actor。"""
    deadline = time.time() + float(timeout)
    while time.time() < deadline:
        actor = find_drone_actor(target_world)
        if actor is not None:
            return actor
        try:
            target_world.wait_for_tick(1.0)
        except Exception:
            time.sleep(0.2)
    return None


def _join_with_timeout(future, seconds, label):
    """等待 AirSim 的 Future，但最多等 seconds 秒。

    AirSim 的 Future.join() 本身没有超时（部分指令如 hoverAsync 也不接受 timeout_sec），
    一旦飞控任务卡住，调用线程会永远停住 —— 界面表现就是"一直显示起飞中，什么都不动"。
    这里用看门狗线程兜底，超时就放弃等待并继续后续流程。
    """
    done = threading.Event()

    def _waiter():
        try:
            future.join()
        except Exception as exc:
            print(f"[Drone] {label} 任务异常: {exc}")
        finally:
            done.set()

    threading.Thread(target=_waiter, daemon=True).start()
    if not done.wait(timeout=seconds):
        print(f"[Drone] {label} 超过 {seconds}s 未完成，不再等待")
        return False
    return True


def _height_above_road_carla():
    """用 CARLA 侧的坐标算无人机离路面的高度（米）。

    这个集成的 AirSim NED 原点会漂移、landed_state 在贴地时也可能停在 Flying，
    只有"无人机 CARLA Z − 脚下路面 Z"是可靠的离地高度。
    """
    if world is None:
        return None
    try:
        actor = find_drone_actor(world) or drone_actor
        if actor is None:
            return None
        location = actor.get_transform().location
        ground_z = _road_ground_z(world, location)
        if ground_z is None:
            return None
        return float(location.z) - float(ground_z)
    except Exception as exc:
        print(f"[Drone] 计算离地高度失败: {exc}")
        return None


def _is_airsim_airborne(ac):
    try:
        # 优先用 CARLA 侧的离地高度：NED 原点漂移和 landed_state 误报都不会影响它。
        height = _height_above_road_carla()
        if height is not None:
            return height > 2.5
        state = ac.getMultirotorState()
        # NED Z 是相对起飞原点的高度，高架桥上的已着陆无人机仍可能为负值。
        # landed_state 由 AirSim 飞控根据接触状态给出，才可用于判断是否已经着陆。
        if airsim is not None:
            return state.landed_state != airsim.LandedState.Landed
        return float(state.kinematics_estimated.position.z_val) < -0.5
    except Exception:
        # 状态读取失败时不能误判为已着陆，否则随后解除动力会造成坠落。
        return True


def _landing_approach_ned_z(ac):
    """计算当前道路层上方 3m 的 AirSim NED 预着陆高度。"""
    if world is None:
        return None
    try:
        actor = find_drone_actor(world) or drone_actor
        if actor is None:
            return None
        actor_location = actor.get_transform().location
        ground_z = _road_ground_z(world, actor_location)
        airsim_z = float(ac.getMultirotorState().kinematics_estimated.position.z_val)
        # CARLA Z 向上、AirSim NED Z 向下；用当前同一架无人机的两套位置消除地图原点偏移。
        target_z = airsim_z + float(actor_location.z) - ground_z - 3.0
        # 仅在目标确实位于当前高度下方时执行预下降，避免道路投影异常导致上升。
        return target_z if target_z > airsim_z + 0.5 else None
    except Exception as exc:
        print(f"[Drone] 无法计算道路层预着陆高度，直接请求 AirSim 着陆: {exc}")
        return None


def _carla_window_is_foreground():
    """判断当前前台窗口是否属于 CarlaAir / CarlaUE4 进程。

    AirSim 的键盘飞行控制本来就要求 UE4 窗口有焦点，因此"人工接管"判定也必须
    限定在仿真器窗口处于前台时，否则用户在浏览器/编辑器里打字就会误判。
    """
    try:
        user32 = ctypes.windll.user32
        kernel32 = ctypes.windll.kernel32
        hwnd = user32.GetForegroundWindow()
        if not hwnd:
            return False
        pid = ctypes.c_ulong(0)
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        if not pid.value:
            return False
        PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
        handle = kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, pid.value)
        if not handle:
            return False
        try:
            buffer = ctypes.create_unicode_buffer(260)
            size = ctypes.c_ulong(260)
            if not kernel32.QueryFullProcessImageNameW(handle, 0, buffer, ctypes.byref(size)):
                return False
            name = os.path.basename(buffer.value).lower()
            return name.startswith('carlaue4') or name.startswith('carlaair')
        finally:
            kernel32.CloseHandle(handle)
    except Exception:
        # 查询失败时退回到"不算人工接管"，避免误中断正在进行的巡航。
        return False


def _pressed_manual_control_key():
    """返回当前按下的 Unreal 飞行控制键（仅当仿真器窗口在前台时才判定）。"""
    keys = (
        (0x57, 'W'), (0x41, 'A'), (0x53, 'S'), (0x44, 'D'),
        (0x51, 'Q'), (0x45, 'E'), (0x20, 'Space'),
        (0x25, 'Left'), (0x26, 'Up'), (0x27, 'Right'), (0x28, 'Down'),
    )
    try:
        user32 = ctypes.windll.user32
        # 关键：只在 CarlaAir 窗口处于前台时才把按键当成人工接管飞行。
        # 否则用户在别的窗口打字（w/a/s/d/空格）就会把自动巡航误判断掉。
        if not _carla_window_is_foreground():
            return None
        for virtual_key, name in keys:
            if user32.GetAsyncKeyState(virtual_key) & 0x8000:
                return name
    except Exception:
        # 非 Windows 环境不支持该保护；AirSim 主流程仍可正常运行。
        pass
    return None


def _ensure_airsim_for_cruise(altitude):
    """连接、接管并让无人机稳定在本次巡航高度。"""
    global airsim_client, airsim_connected, _state_poller_running, _state_poller_thread
    global cruise_state

    if airsim is None:
        raise RuntimeError('airsim 模块未安装')

    if not airsim_connected or airsim_client is None:
        airsim_client = airsim.MultirotorClient(port=41451)
        airsim_client.confirmConnection()

    # 自动巡航期间始终由 API 持有控制权；先清掉可能残留的旧任务，避免和起飞指令打架。
    try:
        airsim_client.cancelLastTask()
    except Exception:
        pass
    airsim_client.enableApiControl(True)
    cruise_state = 'taking_off'
    airsim_client.armDisarm(True)

    height = _height_above_road_carla()
    airborne = _is_airsim_airborne(airsim_client)
    print(f"[Cruise] 起飞前：离地高度={('%.1f' % height) if height is not None else '未知'}m，判定空中={airborne}")
    if not airborne:
        try:
            # 所有飞控指令都带超时，且等待也用看门狗兜底：绝不无限期卡在"起飞中"。
            _join_with_timeout(airsim_client.takeoffAsync(timeout_sec=30), 45, '起飞')
        except Exception as exc:
            cruise_state = 'takeoff_error'
            raise RuntimeError(f'起飞失败: {exc}') from exc
    try:
        target_z = -abs(float(altitude))
        _join_with_timeout(
            airsim_client.moveToZAsync(target_z, 5, timeout_sec=45), 60,
            f'爬升到 {abs(target_z):.0f}m',
        )
    except Exception as exc:
        # 爬升失败不致命：无人机已经在空中，后面的路径任务会自己带着它走。
        print(f"[Cruise] 爬升阶段异常（继续交给路径任务）: {exc}")

    position = airsim_client.getMultirotorState().kinematics_estimated.position
    print(f"[Cruise] 起飞阶段结束：NED z={float(position.z_val):.1f}，离地 {(_height_above_road_carla() or 0):.1f}m")
    airsim_connected = True

    if not _state_poller_running:
        _state_poller_running = True
        _state_poller_thread = threading.Thread(target=_state_poller, daemon=True)
        _state_poller_thread.start()


def _state_poller():
    """后台线程：持续读取无人机状态到缓存，不阻塞任何 API 请求"""
    global _cached_drone_state, _state_poller_running
    global airsim_client, airsim_connected, cruise_running, cruise_controller
    print("[Drone] 状态轮询线程启动")
    while _state_poller_running:
        try:
            if airsim_client is not None and airsim_connected:
                state = airsim_client.getMultirotorState()
                pos = state.kinematics_estimated.position
                wp_total = len(cruise_controller.route.waypoints) if (cruise_controller and cruise_controller.route) else 0
                wp_idx = cruise_controller.index if cruise_controller else 0
                # 巡航任务只提交一次，AirSim 不会回报“走到第几个航点”；
                # 这里用当前坐标反查最近航点，给出真实的路径进度。
                if (cruise_controller is not None and cruise_controller.route is not None
                        and wp_total > 0 and nearest_route_index is not None):
                    nearest = nearest_route_index(
                        cruise_controller.route.waypoints,
                        float(pos.x_val), float(pos.y_val),
                    )
                    if nearest is not None:
                        wp_idx = nearest
                        cruise_controller.index = nearest
                _cached_drone_state = {
                    'x': round(float(pos.x_val), 1),
                    'y': round(float(pos.y_val), 1),
                    'z': round(float(pos.z_val), 1),
                    'wp_index': wp_idx,
                    'wp_total': wp_total,
                    'ok': True, 'error': '',
                }
            else:
                _cached_drone_state['ok'] = False
        except Exception as e:
            _cached_drone_state = {
                'x': 0, 'y': 0, 'z': 0,
                'wp_index': 0, 'wp_total': 0,
                'ok': False, 'error': str(e)[:100],
            }
        time.sleep(0.5)
    print("[Drone] 状态轮询线程退出")


def _port_is_open(port, host='127.0.0.1', timeout=0.5):
    """快速探测本机端口是否在监听（不占用 carla_lock）。"""
    import socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            return sock.connect_ex((host, int(port))) == 0
    except Exception:
        return False


def _carla_connect_watchdog():
    """后台线程：仿真器比后端后启动时自动附着。

    场景：用户双击「启动 Carla-Air 仿真器」快捷方式（或自己开 StartCarlaAir.bat），
    而后端/界面早就在跑。没有这个线程时，用户还得回界面点一次「连接已有仿真器」。
    """
    global _carla_watchdog_running, airsim_client, airsim_connected
    global _state_poller_thread, _state_poller_running
    print('[Watchdog] 自动附着视线程启动')
    while _carla_watchdog_running:
        try:
            if (not carla_ready
                    and time.time() >= _carla_watchdog_paused_until
                    and _port_is_open(2000)):
                with carla_lock:
                    if not carla_ready:
                        print('[Watchdog] 检测到 CARLA 端口 2000 已监听，自动连接 ...')
                        if connect_carla():
                            if not airsim_connected or airsim_client is None:
                                try:
                                    airsim_client = airsim.MultirotorClient(port=41451)
                                    airsim_client.confirmConnection()
                                    airsim_connected = True
                                except Exception as airsim_error:
                                    airsim_connected = False
                                    print(f'[Watchdog] AirSim 暂时连不上: {airsim_error}')
                            if not _state_poller_running:
                                _state_poller_running = True
                                _state_poller_thread = threading.Thread(target=_state_poller, daemon=True)
                                _state_poller_thread.start()
                            print(f'[Watchdog] 已自动附着到 {current_map_name}')
        except Exception as exc:
            print(f'[Watchdog] 异常（忽略并继续）: {exc}')
        time.sleep(4)


def _cruise_loop():
    """仅监测人工接管，完整路径由 AirSim 的单个飞行任务持续执行。"""
    global cruise_running, cruise_controller, manual_override_id, manual_override_key
    print("[Cruise] 巡航监测线程启动")
    while True:
        with cruise_lock:
            if not cruise_running or cruise_controller is None:
                break
        key = _pressed_manual_control_key()
        if key:
            # Python 无法直接屏蔽 Unreal 的原始键盘输入，将其定义为退出自动巡航。
            with cruise_lock:
                manual_override_id += 1
                manual_override_key = key
            print(f"[Cruise] 检测到用户飞行输入 {key}，退出自动巡航")
            _stop_cruise(release_control=True, state_after='manual_override')
            break
        time.sleep(0.2)
    print("[Cruise] 巡航线程退出")


def _stop_cruise(release_control=False, state_after=None):
    """停止唯一飞控线程；暂停后才将键盘控制权交还给用户。"""
    global cruise_running, cruise_thread, cruise_controller, cruise_state

    with cruise_lock:
        cruise_running = False
        thread = cruise_thread

    if thread is not None and thread is not threading.current_thread():
        thread.join(timeout=3)

    if airsim_client is not None:
        try:
            airsim_client.cancelLastTask()
        except Exception:
            pass
        # cancelLastTask 会取消单个路径跟随任务；此处不再另发并等待刹停任务，
        # 避免在异常收尾时阻塞后续的 landAsync。
        if release_control:
            try:
                airsim_client.enableApiControl(False)
            except Exception:
                pass

    with cruise_lock:
        cruise_thread = None
        cruise_controller = None
        cruise_state = state_after or ('manual' if release_control else 'paused')


def _land_drone():
    """停止巡航后在当前位置稳定着陆，确认落地前不得解除动力。"""
    global airsim_connected, _state_poller_running, _state_poller_thread, cruise_state

    try:
        _stop_cruise(release_control=False)
    except Exception as exc:
        # 任务取消失败时仍继续尝试降落，不能让清理链路在第一步中断。
        print(f"[Drone] 停止巡航时出现异常，继续降落: {exc}")
    _state_poller_running = False
    if _state_poller_thread is not None and _state_poller_thread is not threading.current_thread():
        _state_poller_thread.join(timeout=2)
    _state_poller_thread = None

    try:
        if airsim_client is not None and _is_airsim_airborne(airsim_client):
            cruise_state = 'landing'
            # 点过「暂停」时控制权已交还给用户（enableApiControl(False)），
            # 此时直接发降落指令会被飞控忽略，必须先重新接管再降落。
            try:
                airsim_client.enableApiControl(True)
                airsim_client.armDisarm(True)
            except Exception as takeover_error:
                print(f"[Drone] 降落前重新接管控制权失败（继续尝试降落）: {takeover_error}")
            # 取消路径任务后先稳定当前姿态，避免带着水平速度直接开始下降。
            # hoverAsync 不接受 timeout_sec，所以用看门狗兜底，避免卡在这里。
            _join_with_timeout(airsim_client.hoverAsync(), 15, '悬停稳定')
            try:
                approach_z = _landing_approach_ned_z(airsim_client)
                if approach_z is not None:
                    # 先到当前道路层上方 3m，再着陆；桥面和坡道不会被误当作地面以下。
                    print(f"[Drone] 下降至道路层上方 3m: NED_z={approach_z:.2f}")
                    airsim_client.moveToZAsync(approach_z, 3.0, timeout_sec=60).join()
            except Exception as approach_error:
                # 高度收敛失败不能阻断着陆：直接进入 landAsync，由飞控自己下沉。
                print(f"[Drone] 下降至道路层失败，直接着陆: {approach_error}")
            print("[Drone] 原地自动着陆...")
            land_task = airsim_client.landAsync(timeout_sec=90)
            land_task.join()
            # landAsync 的返回值由 AirSim 飞控在着陆任务完成后给出。
            # 本集成的 landed_state 会在桥面和地面上长期停留在 Flying，不能作为失败条件。
            land_result = land_task.get() if hasattr(land_task, 'get') else True
            if land_result is False:
                raise RuntimeError('AirSim 着陆任务返回失败')
    except Exception as exc:
        cruise_state = 'landing_error'
        print(f"[Drone] 自动着陆失败: {exc}")
        # 着陆失败时保持动力和 API 控制权，不能让无人机失去升力而坠落。
        raise RuntimeError(f'自动着陆失败: {exc}') from exc

    if airsim_client is not None:
        try:
            airsim_client.armDisarm(False)
            airsim_client.enableApiControl(False)
        except Exception as exc:
            print(f"[Drone] 释放控制权失败: {exc}")
    airsim_connected = False
    cruise_state = 'landed'
    print("[Drone] 已降落并释放控制权")


def _start_road_cruise(speed, altitude, route_save_dir, route_id=None):
    """创建路线并启动唯一的自动巡航控制线程；传入 route_id 时使用已保存的自定义航线。"""
    global cruise_controller, cruise_thread, cruise_running, cruise_state
    global cruise_altitude, drone_actor, manual_override_key
    global airsim_client, airsim_connected

    if world is None:
        raise RuntimeError('CARLA 未就绪')
    if load_fixed_cruise_route is None:
        raise RuntimeError('road_cruise 模块未加载')
    if route_id and load_custom_cruise_route is None:
        raise RuntimeError('自定义航线模块未加载')

    with cruise_lock:
        if cruise_running:
            raise RuntimeError('巡航已在进行中')
        manual_override_key = ''
        cruise_state = 'preparing'

    drone_actor = _wait_for_drone_actor(world)
    if drone_actor is None:
        raise RuntimeError('未找到无人机 CARLA actor')

    if route_id:
        cruise_state = 'loading_custom_route'
        route = load_custom_cruise_route(world.get_map().name, route_id)
    else:
        cruise_state = 'loading_fixed_route'
        # 航线文件是经过测试的 AirSim 坐标资产；巡航时禁止重新计算 CARLA 拓扑或坐标偏移。
        route = load_fixed_cruise_route(world.get_map().name)
    if not airsim_connected or airsim_client is None:
        airsim_client = airsim.MultirotorClient(port=41451)
        airsim_client.confirmConnection()
    airsim_pos = airsim_client.getMultirotorState().kinematics_estimated.position
    first = route.waypoints[0]
    first_distance = ((first.x - float(airsim_pos.x_val)) ** 2
                      + (first.y - float(airsim_pos.y_val)) ** 2) ** 0.5
    if first_distance > 35.0:
        if route_id:
            # 自定义航线的起点由用户在图上点击决定，允许补一段连接段飞到起点（避免直线穿楼），
            # 但连接段只在合理距离内才安全，超了就明确告诉用户怎么办，而不是报"坐标系不匹配"。
            if first_distance <= 200.0 and prepend_start_connector is not None:
                route = prepend_start_connector(
                    route, float(airsim_pos.x_val), float(airsim_pos.y_val),
                )
                first = route.waypoints[0]
                first_distance = ((first.x - float(airsim_pos.x_val)) ** 2
                                  + (first.y - float(airsim_pos.y_val)) ** 2) ** 0.5
                print(f"[Cruise] 自定义航线起点距无人机 {first_distance:.1f}m，已补连接段")
            else:
                raise RuntimeError(
                    f'自定义航线起点距无人机 {first_distance:.0f}m，超过 200m 的连接段上限。'
                    '请先把无人机飞到起点附近，或在「地图与路线」页把起点选在无人机当前位置附近，再开始巡航。'
                )
        else:
            # 固定航线：无人机可能停在航线中段（例如上一次巡航点过「暂停」后就地降落）。
            # 这时从最近的航点接着飞到航线终点即可：往返航线的终点≈起点，
            # 既不用为了回到起点而直线穿楼，下一次巡航又能从头开始。
            resume_index = None
            if nearest_route_index is not None:
                nearest = nearest_route_index(
                    route.waypoints, float(airsim_pos.x_val), float(airsim_pos.y_val),
                )
                if nearest is not None:
                    waypoint = route.waypoints[nearest]
                    nearest_distance = math.hypot(
                        waypoint.x - float(airsim_pos.x_val),
                        waypoint.y - float(airsim_pos.y_val),
                    )
                    if 0 < nearest < len(route.waypoints) - 2 and nearest_distance <= 35.0:
                        resume_index = nearest
            if resume_index is None:
                raise RuntimeError(
                    f'航线首航点距无人机 {first_distance:.1f}m，'
                    '当前地图或 AirSim 坐标系与该航线不匹配'
                )
            route = CruiseRoute(
                map_name=route.map_name,
                source=f'{route.source}+resume_from_{resume_index}',
                waypoints=list(route.waypoints[resume_index:]),
                profile=route.profile,
            )
            first = route.waypoints[0]
            first_distance = math.hypot(
                first.x - float(airsim_pos.x_val),
                first.y - float(airsim_pos.y_val),
            )
            print(f"[Cruise] 无人机停在航线中段，从第 {resume_index} 个航点接着飞"
                  f"（距该航点 {first_distance:.1f}m）")
    fixed_altitude = abs(float(route.waypoints[0].z))
    _ensure_airsim_for_cruise(fixed_altitude)
    print(f"[Cruise] 已加载航线: {route.source}, 高度={fixed_altitude:.1f}m")

    controller = RoadCruiseController(airsim_client, route, speed=float(speed))
    # 只调用一次 moveOnPathAsync。之后 AirSim 自行沿固定路径飞行，
    # Python 线程不再每隔一段时间创建新的移动任务。
    controller.start()
    monitor_thread = threading.Thread(
        target=_cruise_loop, daemon=True,
    )
    with cruise_lock:
        cruise_controller = controller
        cruise_altitude = float(altitude)
        cruise_state = 'auto_cruise'
        cruise_running = True
        cruise_thread = monitor_thread
        monitor_thread.start()

    print(f"[Cruise] 启动成功: map={route.map_name}, waypoints={len(route.waypoints)}, speed={speed}")
    return route


@app.route('/api/drone/connect', methods=['POST'])
def drone_connect():
    """连接 AirSim 并起飞"""
    global airsim_client, airsim_connected, drone_actor, cruise_altitude, cruise_state
    global _state_poller_thread, _state_poller_running
    with carla_lock:
        try:
            if airsim is None:
                return jsonify({'success': False, 'error': 'airsim 模块未安装'}), 500
            if drone_actor is None:
                drone_actor = find_drone_actor(world)
            if drone_actor is None:
                return jsonify({'success': False, 'error': '未找到无人机 CARLA actor', 'hint': '请先在仿真器页面启动 CARLA-Air'}), 500

            data = request.get_json(silent=True) or {}
            port = data.get('airsim_port', 41451)
            altitude = data.get('altitude', 45)

            airsim_client = airsim.MultirotorClient(port=port)
            airsim_client.confirmConnection()
            airsim_client.enableApiControl(True)
            airsim_client.armDisarm(True)
            print("[Drone] 起飞中...")
            airsim_client.takeoffAsync().join()
            airsim_client.moveToZAsync(-float(altitude), 5).join()
            airsim_connected = True
            cruise_altitude = float(altitude)
            cruise_state = 'manual'

            # 启动状态轮询后台线程（永不阻塞 API）
            _state_poller_running = True
            _state_poller_thread = threading.Thread(target=_state_poller, daemon=True)
            _state_poller_thread.start()

            print(f"[Drone] AirSim 连接成功, 已起飞至 {altitude}m")
            return jsonify({'success': True, 'message': f'已连接, 高度 {altitude}m'})
        except Exception as e:
            airsim_connected = False
            print(f"[Drone] 连接失败: {e}")
            return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/drone/status', methods=['GET'])
def drone_status():
    """获取无人机状态 — 只读缓存，永不阻塞"""
    s = _cached_drone_state
    return jsonify({
        'connected': airsim_connected,
        'airsim_ok': s['ok'],
        'cruising': cruise_running,
        'control_state': cruise_state,
        'manual_override': {
            'id': manual_override_id,
            'key': manual_override_key,
        } if manual_override_id else None,
        'position': {'x': s['x'], 'y': s['y'], 'z': s['z']} if s['ok'] else None,
        'waypoint_index': s['wp_index'],
        'total_waypoints': s['wp_total'],
    })


@app.route('/api/drone/cruise/start', methods=['POST'])
def drone_cruise_start():
    """自动连接无人机 + 起飞 + 生成路线 + 启动巡航（一键完成）"""
    with carla_lock:
        try:
            # 所有启动入口复用同一飞控线程，避免与采集循环重复发移动命令。
            data = request.get_json(silent=True) or {}
            speed = float(data.get('speed', 8))
            altitude = float(data.get('altitude', 45))
            save_dir = data.get('route_save_dir', './recordings/drone_routes')
            route = _start_road_cruise(speed, altitude, save_dir, data.get('route_id'))
            return jsonify({
                'success': True,
                'map_name': route.map_name,
                'waypoints_count': len(route.waypoints),
                'source': route.source,
                'auto_connected': True,
            })

        except Exception as e:
            _stop_cruise(release_control=True)
            print(f"[Cruise] 启动失败: {e}")
            return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/drone/cruise/stop', methods=['POST'])
def drone_cruise_stop():
    """停止巡航，无人机悬停"""
    try:
        _stop_cruise(release_control=True)
        return jsonify({'success': True, 'message': '巡航已暂停，控制权已交还用户'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/drone/land', methods=['POST'])
def drone_land():
    """停止巡航后在当前位置自动降落。"""
    try:
        _land_drone()
        return jsonify({'success': True, 'message': '已降落'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


if __name__ == '__main__':
    carla_path_arg = None
    args_list = []
    for arg in sys.argv[1:]:
        if arg.startswith('--'):
            args_list.append(arg)
        else:
            carla_path_arg = arg

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5000)
    args = parser.parse_args(args_list)

    if carla_path_arg:
        print(f"命令行参数 Carla-Air 路径: {carla_path_arg}")

    print(f"API 服务器启动: http://localhost:{args.port}")
    if carla_air_dir:
        print(f"Carla-Air 路径: {carla_air_dir}")
    else:
        print("警告: Carla-Air 路径未配置")

    # 后端起得比仿真器早时，自动附着（用户点完「启动仿真器」不用再回界面点连接）。
    _carla_watchdog_thread = threading.Thread(target=_carla_connect_watchdog, daemon=True)
    _carla_watchdog_thread.start()

    app.run(host='127.0.0.1', port=args.port, debug=False, threaded=True)
