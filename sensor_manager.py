"""
传感器管理器：孵化、采集、销毁 CARLA 传感器
同步模式专用：sensor_tick = 0.0，每 tick 一帧
"""
import os
import json
import carla
import numpy as np
from PIL import Image
import airsim
import concurrent.futures
import io
import time

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)

def save_numpy(save_dir, frame_idx, data, ext='npy'):
    ensure_dir(save_dir)
    filename = os.path.join(save_dir, f"frame_{frame_idx:06d}.{ext}")
    np.save(filename, data)

def save_json(save_dir, frame_idx, data):
    ensure_dir(save_dir)
    filename = os.path.join(save_dir, f"frame_{frame_idx:06d}.json")
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)

# ==================== CARLA RGB 相机 ====================
def make_rgb_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(image):
        idx = counter[0]
        if idx >= max_frames:
            return
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        rgb = array[:, :, [2,1,0]]
        im = Image.fromarray(rgb)
        filename = os.path.join(save_dir, f"frame_{idx:06d}.png")
        im.save(filename)
        counter[0] += 1
    return callback

def spawn_rgb_camera(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
    blueprint.set_attribute('image_size_x', str(config['params'].get('width', 800)))
    blueprint.set_attribute('image_size_y', str(config['params'].get('height', 600)))
    blueprint.set_attribute('fov', str(config['params'].get('fov', 90)))
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== CARLA 深度相机 ====================
def make_depth_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(image):
        idx = counter[0]
        if idx >= max_frames:
            return
        array = np.frombuffer(image.raw_data, dtype=np.float32)
        array = array.reshape((image.height, image.width))
        array = np.nan_to_num(array, nan=0.0, posinf=0.0, neginf=0.0)
        depth_mm = (array * 1000).astype(np.uint16)
        im = Image.fromarray(depth_mm, mode='I;16')
        filename = os.path.join(save_dir, f"frame_{idx:06d}.png")
        im.save(filename)
        counter[0] += 1
    return callback

def spawn_depth_camera(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.camera.depth')
    blueprint.set_attribute('image_size_x', str(config['params'].get('width', 800)))
    blueprint.set_attribute('image_size_y', str(config['params'].get('height', 600)))
    blueprint.set_attribute('fov', str(config['params'].get('fov', 90)))
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== CARLA 语义分割相机 ====================
def make_semantic_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(image):
        idx = counter[0]
        if idx >= max_frames:
            return
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        semantic = array[:, :, 2]
        im = Image.fromarray(semantic.astype(np.uint8), mode='L')
        filename = os.path.join(save_dir, f"frame_{idx:06d}.png")
        im.save(filename)
        counter[0] += 1
    return callback

def spawn_semantic_camera(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
    blueprint.set_attribute('image_size_x', str(config['params'].get('width', 800)))
    blueprint.set_attribute('image_size_y', str(config['params'].get('height', 600)))
    blueprint.set_attribute('fov', str(config['params'].get('fov', 90)))
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== CARLA 实例分割相机 ====================
def make_instance_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(image):
        idx = counter[0]
        if idx >= max_frames:
            return
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        tag = array[:, :, 2].astype(np.uint16)
        instance_id = array[:, :, 1].astype(np.uint16)
        combined = (tag.astype(np.uint32) << 16) | instance_id.astype(np.uint32)
        filename = os.path.join(save_dir, f"frame_{idx:06d}.npy")
        np.save(filename, combined)
        counter[0] += 1
    return callback

def spawn_instance_camera(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.camera.instance_segmentation')
    blueprint.set_attribute('image_size_x', str(config['params'].get('width', 800)))
    blueprint.set_attribute('image_size_y', str(config['params'].get('height', 600)))
    blueprint.set_attribute('fov', str(config['params'].get('fov', 90)))
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== CARLA 法线相机 ====================
def make_normals_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(image):
        idx = counter[0]
        if idx >= max_frames:
            return
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        normals = array[:, :, :3][:, :, [2, 1, 0]]
        im = Image.fromarray(normals)
        filename = os.path.join(save_dir, f"frame_{idx:06d}.png")
        im.save(filename)
        counter[0] += 1
    return callback

def spawn_normals_camera(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.camera.normals')
    blueprint.set_attribute('image_size_x', str(config['params'].get('width', 800)))
    blueprint.set_attribute('image_size_y', str(config['params'].get('height', 600)))
    blueprint.set_attribute('fov', str(config['params'].get('fov', 90)))
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== CARLA 光流相机 ====================
def make_optical_flow_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(image):
        idx = counter[0]
        if idx >= max_frames:
            return
        array = np.frombuffer(image.raw_data, dtype=np.float32)
        array = array.reshape((image.height, image.width, 2))
        filename = os.path.join(save_dir, f"frame_{idx:06d}.npy")
        np.save(filename, array)
        counter[0] += 1
    return callback

def spawn_optical_flow_camera(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.camera.optical_flow')
    blueprint.set_attribute('image_size_x', str(config['params'].get('width', 800)))
    blueprint.set_attribute('image_size_y', str(config['params'].get('height', 600)))
    blueprint.set_attribute('fov', str(config['params'].get('fov', 90)))
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== CARLA LiDAR ====================
def make_lidar_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(point_cloud):
        idx = counter[0]
        if idx >= max_frames:
            return
        data = np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32)
        ]))
        points = np.array([(d['x'], d['y'], d['z'], d['intensity']) for d in data])
        filename = os.path.join(save_dir, f"frame_{idx:06d}.npy")
        np.save(filename, points)
        counter[0] += 1
    return callback

def spawn_lidar(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    blueprint.set_attribute('channels', str(config['params'].get('channels', 32)))
    blueprint.set_attribute('range', str(config['params'].get('range', 100)))
    blueprint.set_attribute('points_per_second', str(config['params'].get('points_per_second', 56000)))
    blueprint.set_attribute('rotation_frequency', str(config['params'].get('rotation_frequency', 10)))
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== CARLA 语义 LiDAR ====================
def make_semantic_lidar_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(point_cloud):
        idx = counter[0]
        if idx >= max_frames:
            return
        data = np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32), ('semantic_tag', np.uint32), ('instance_tag', np.uint32)
        ]))
        points = np.array([(d['x'], d['y'], d['z'], d['intensity'], d['semantic_tag'], d['instance_tag']) for d in data])
        filename = os.path.join(save_dir, f"frame_{idx:06d}.npy")
        np.save(filename, points)
        counter[0] += 1
    return callback

def spawn_semantic_lidar(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
    blueprint.set_attribute('channels', str(config['params'].get('channels', 32)))
    blueprint.set_attribute('range', str(config['params'].get('range', 100)))
    blueprint.set_attribute('points_per_second', str(config['params'].get('points_per_second', 56000)))
    blueprint.set_attribute('rotation_frequency', str(config['params'].get('rotation_frequency', 10)))
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== CARLA 雷达 ====================
def make_radar_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(radar_data):
        idx = counter[0]
        if idx >= max_frames:
            return
        points = []
        for detection in radar_data:
            points.append({
                'velocity': detection.velocity,
                'altitude': detection.altitude,
                'azimuth': detection.azimuth,
                'depth': detection.depth
            })
        filename = os.path.join(save_dir, f"frame_{idx:06d}.json")
        with open(filename, 'w') as f:
            json.dump(points, f, indent=2)
        counter[0] += 1
    return callback

def spawn_radar(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.other.radar')
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== CARLA GNSS ====================
def make_gnss_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(gnss_data):
        idx = counter[0]
        if idx >= max_frames:
            return
        data = {
            'latitude': gnss_data.latitude,
            'longitude': gnss_data.longitude,
            'altitude': gnss_data.altitude
        }
        filename = os.path.join(save_dir, f"frame_{idx:06d}.json")
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        counter[0] += 1
    return callback

def spawn_gnss(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.other.gnss')
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== CARLA IMU ====================
def make_imu_callback(save_dir, max_frames):
    ensure_dir(save_dir)
    counter = [0]
    def callback(imu_data):
        idx = counter[0]
        if idx >= max_frames:
            return
        data = {
            'accelerometer': {'x': imu_data.accelerometer.x, 'y': imu_data.accelerometer.y, 'z': imu_data.accelerometer.z},
            'gyroscope': {'x': imu_data.gyroscope.x, 'y': imu_data.gyroscope.y, 'z': imu_data.gyroscope.z},
            'compass': imu_data.compass
        }
        filename = os.path.join(save_dir, f"frame_{idx:06d}.json")
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        counter[0] += 1
    return callback

def spawn_imu(world, drone, config):
    blueprint = world.get_blueprint_library().find('sensor.other.imu')
    blueprint.set_attribute('sensor_tick', '0.0')
    transform = carla.Transform(
        carla.Location(**config['position']),
        carla.Rotation(**config['rotation'])
    )
    sensor = world.spawn_actor(blueprint, transform, attach_to=drone)
    return sensor

# ==================== 回调映射 ====================
CALLBACK_MAP = {
    'camera_rgb': make_rgb_callback,
    'camera_depth': make_depth_callback,
    'camera_semantic': make_semantic_callback,
    'camera_instance': make_instance_callback,
    'camera_optical_flow': make_optical_flow_callback,
    'camera_normals': make_normals_callback,
    'lidar': make_lidar_callback,
    'semantic_lidar': make_semantic_lidar_callback,
    'radar': make_radar_callback,
    'gnss': make_gnss_callback,
    'imu': make_imu_callback,
}

def start_listeners(active_sensors, save_dir, max_frames):
    print(f"[Listener] 准备为 {len(active_sensors)} 个传感器注册回调")
    for sensor, sensor_id in active_sensors:
        if sensor_id not in CALLBACK_MAP:
            print(f"警告：未找到传感器 {sensor_id} 的回调，跳过")
            continue
        sub_dir = os.path.join(save_dir, sensor_id)
        callback = CALLBACK_MAP[sensor_id](sub_dir, max_frames)
        sensor.listen(callback)
        print(f"[Listener] 已为 {sensor_id} 注册回调 -> {sub_dir}")

def destroy_sensors(sensor_list):
    print(f"[Destroy] 正在销毁 {len(sensor_list)} 个传感器")
    for sensor, sensor_id in sensor_list:
        if sensor is not None and sensor.is_alive:
            sensor.destroy()
            print(f"[Destroy] 已销毁 {sensor_id}")
    sensor_list.clear()

# ==================== AirSim 支持 ====================
def generate_airsim_settings(settings_path, params, sensors_dict):
    """
    根据用户参数和传感器启用状态生成 settings.json
    params 包含: width, height, fov, lidarChannels, lidarRange, lidarPointsPerSecond, lidarRotationFrequency
    sensors_dict: { sensor_id: enabled, ... }
    """
    width = params.get('width', 800)
    height = params.get('height', 600)
    fov = params.get('fov', 90)
    lidar_channels = params.get('lidarChannels', 16)
    lidar_range = params.get('lidarRange', 100)
    lidar_pps = params.get('lidarPointsPerSecond', 100000)
    lidar_rpm = params.get('lidarRotationFrequency', 10)

    imu_en = sensors_dict.get('imu_airsim', False)
    gps_en = sensors_dict.get('gps_airsim', False)
    baro_en = sensors_dict.get('barometer_airsim', False)
    mag_en = sensors_dict.get('magnetometer_airsim', False)
    lidar_en = sensors_dict.get('lidar_airsim', False)

    settings = {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "Vehicles": {
            "SimpleFlight": {
                "VehicleType": "SimpleFlight",
                "AutoCreate": True,
                "Cameras": {
                    "0": {
                        "CaptureSettings": [
                            {"ImageType": 0, "Width": width, "Height": height, "FOV_Degrees": fov},
                            {"ImageType": 1, "Width": width, "Height": height, "FOV_Degrees": fov},
                            {"ImageType": 2, "Width": width, "Height": height, "FOV_Degrees": fov},
                            {"ImageType": 3, "Width": width, "Height": height, "FOV_Degrees": fov},
                            {"ImageType": 4, "Width": width, "Height": height, "FOV_Degrees": fov},
                            {"ImageType": 5, "Width": width, "Height": height, "FOV_Degrees": fov},
                            {"ImageType": 6, "Width": width, "Height": height, "FOV_Degrees": fov},
                            {"ImageType": 7, "Width": width, "Height": height, "FOV_Degrees": fov},
                            {"ImageType": 8, "Width": width, "Height": height, "FOV_Degrees": fov},
                            {"ImageType": 9, "Width": width, "Height": height, "FOV_Degrees": fov}
                        ],
                        "X": 0.0, "Y": 0.0, "Z": 1.0,
                        "Pitch": -15.0, "Roll": 0.0, "Yaw": 0.0
                    }
                },
                "Sensors": {
                    "LidarSensor": {
                        "SensorType": 6,
                        "Enabled": lidar_en,
                        "NumberOfChannels": lidar_channels,
                        "PointsPerSecond": lidar_pps,
                        "X": 0.0, "Y": 0.0, "Z": -0.3,
                        "RotationsPerSecond": lidar_rpm,
                        "Range": lidar_range,
                        "DrawDebugPoints": False
                    }
                }
            }
        },
        "DefaultSensors": {
            "Imu": {"SensorType": 2, "Enabled": imu_en},
            "Gps": {"SensorType": 3, "Enabled": gps_en},
            "Barometer": {"SensorType": 1, "Enabled": baro_en},
            "Magnetometer": {"SensorType": 4, "Enabled": mag_en}
        }
    }

    with open(settings_path, 'w', encoding='utf-8') as f:
        json.dump(settings, f, indent=4)

    print(f"[AirSim] settings.json 已生成，路径: {settings_path}")

# ==================== 采集 AirSim 帧（带超时与诊断） ====================
def collect_airsim_frame(client, sensors_full_config, enabled_ids, save_dir, frame_idx):
    """
    采集一帧 AirSim 数据（全局连接 + 延时 + 超时）
    """
    if not client:
        print("[AirSim] 全局客户端为空，跳过")
        return {}

    ensure_dir(save_dir)

    image_type_map = {
        "airsim_scene": 0,
        "airsim_depth_planar": 1,
        "airsim_depth_perspective": 2,
        "airsim_depth_vis": 3,
        "airsim_disparity_normalized": 4,
        "airsim_segmentation": 5,
        "airsim_surface_normals": 6,
        "airsim_infrared": 7,
        "airsim_optical_flow": 8,
        "airsim_optical_flow_vis": 9,
    }

    camera_ids = [sid for sid in enabled_ids if sid in image_type_map]
    nav_ids = [sid for sid in enabled_ids if sid in ['imu_airsim', 'gps_airsim', 'barometer_airsim', 'magnetometer_airsim']]

    # 先测试连接
    try:
        client.ping()
    except Exception as e:
        print(f"[AirSim] 第 {frame_idx} 帧 Ping 失败: {e}")
        return {}

    # 关键：给 AirSim 一点时间生成图像（因为 Carla 同步模式可能需额外时间）
    time.sleep(0.2)

    # 处理图像
    if camera_ids:
        requests = []
        for sid in camera_ids:
            img_type = image_type_map[sid]
            requests.append(airsim.ImageRequest("0", img_type, False, True))

        # 使用线程池设置超时，但不阻塞
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        future = executor.submit(client.simGetImages, requests)
        try:
            responses = future.result(timeout=5.0)  # 延长到 5 秒
        except concurrent.futures.TimeoutError:
            print(f"[AirSim] 第 {frame_idx} 帧 simGetImages 超时（5秒），跳过")
            responses = None
        finally:
            executor.shutdown(wait=False)  # 不等待任务结束

        if responses:
            for resp, sid in zip(responses, camera_ids):
                if resp and resp.image_data_uint8:
                    sub_dir = os.path.join(save_dir, sid)
                    ensure_dir(sub_dir)
                    try:
                        with open(os.path.join(sub_dir, f"frame_{frame_idx:06d}.png"), "wb") as f:
                            f.write(resp.image_data_uint8)
                        print(f"[AirSim] 成功保存 {sid} 第 {frame_idx} 帧")
                    except Exception as e:
                        print(f"[AirSim] 保存 {sid} 第 {frame_idx} 帧失败: {e}")
                else:
                    print(f"[AirSim] {sid} 第 {frame_idx} 帧响应为空或无效")
        else:
            print("[AirSim] 未收到有效响应")

    # 导航传感器（直接调用，一般较快）
    for sid in nav_ids:
        sub_dir = os.path.join(save_dir, sid)
        ensure_dir(sub_dir)
        try:
            if sid == 'imu_airsim':
                data = client.getImuData()
                out = {
                    'accelerometer': {'x': data.accelerometer.x_val, 'y': data.accelerometer.y_val, 'z': data.accelerometer.z_val},
                    'gyroscope': {'x': data.gyroscope.x_val, 'y': data.gyroscope.y_val, 'z': data.gyroscope.z_val},
                    'compass': data.compass
                }
            elif sid == 'gps_airsim':
                data = client.getGpsData()
                out = {'latitude': data.latitude, 'longitude': data.longitude, 'altitude': data.altitude}
            elif sid == 'barometer_airsim':
                data = client.getBarometerData()
                out = {'altitude': data.altitude, 'pressure': data.pressure}
            elif sid == 'magnetometer_airsim':
                data = client.getMagnetometerData()
                out = {'magnetic_field': {'x': data.magnetic_field.x_val, 'y': data.magnetic_field.y_val, 'z': data.magnetic_field.z_val}}
            else:
                continue
            with open(os.path.join(sub_dir, f"frame_{frame_idx:06d}.json"), 'w') as f:
                json.dump(out, f, indent=2)
        except Exception as e:
            print(f"[AirSim] 导航传感器 {sid} 第 {frame_idx} 帧采集失败: {e}")

    return {}