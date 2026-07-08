#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
森林防火检测系统 - RK3588 NPU 重构版 v2 (40.py, 基于 39.py + 本地预览窗口)

相比 39.py 的变化:
  1. 采集端主动限速到 CAMERA_TARGET_FPS (借鉴 x4.py), 摄像头照读但不
     无限往 NPU 池灌帧, 避免 pool backlog 无限堆积 -> 内存暴涨 -> 卡死
  2. 主线程加 cv2.imshow 本地预览窗口 (借鉴 x4.py), 直接看到画面
  3. 帧队列语义统一: 满则 get 旧 + put 新, 永远保留最新一帧
  4. 支持无桌面环境自动跳过预览 (headless)

相比 37.py 的核心变化 (39.py 已完成, 保留):
  - 推理: ultralytics.YOLO (CPU) -> RKNNLite × 3 NPU 核 (RKNNPool)
  - 架构: capture / consume / rtp_sender / fire_control 全部解耦, 队列 size=1
  - 中文绘制: PIL 字体缓存, 不再每次重载
  - 串口: /dev/ttyTHS1 -> /dev/ttyS9 (RK3588 默认)
  - 业务(MQTT/串口协议/状态机)与 37.py 语义一致

启动前提:
  models/best.rknn      彩色摄像头模型 (17 类)
  models/best_ir.rknn   热红外摄像头模型 (1 类)

按 q 或 ESC 键关闭程序 (只在预览窗口有焦点时). 无桌面环境自动跳过窗口.
"""
import os
os.environ['OPENCV_LOG_LEVEL'] = 'SILENT'
os.environ['OPENCV_VIDEOIO_DEBUG'] = '0'
# 给 OpenCV/numpy 留两个 CPU 核, NPU 池自己管调度
os.environ.setdefault('OMP_NUM_THREADS', '2')
os.environ.setdefault('MKL_NUM_THREADS', '2')

# ★ 主进程整体降低优先级 (nice +5), 避免抢 GUI/SSH 的 CPU 导致"看起来卡死".
# 39.py 常态跑 20 个线程 + NPU 池, 会持续占满一个 A76 大核, 如果不 nice 一下
# 后台跑的时候 VSCode/Chrome/gnome-shell 会很卡, 用户以为板子挂了.
# nice 5 只是提示调度器"我不急", 对推理 FPS 几乎没影响 (NPU 是硬件加速, 不跟 CPU 抢).
try:
    os.nice(5)
except Exception:
    pass

import datetime
import json
import queue
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from typing import Optional

import cv2
cv2.setLogLevel(0)
import numpy as np
import serial
from paho.mqtt import client as mqtt_client

# 把 rk_runtime 加入 sys.path
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from rk_runtime.rknn_pool import RKNNPool
from rk_runtime.yolov8_postprocess import (INPUT_SIZE, decode_yolov8, letterbox,
                                            nms_per_class, preprocess, scale_back)
from rk_runtime.zh_text import draw_chinese_text

# ============= 输出过滤：只显示 fly/i2c 相关日志（保留原项目行为）=============
import builtins
_real_print = builtins.print

def print(*args, **kwargs):
    msg = ' '.join(str(a) for a in args)
    if 'fly' in msg.lower() or 'i2c' in msg.lower():
        _real_print(*args, **kwargs)

builtins.print = print

# ============= 配置参数 =============
# MQTT (华为云 IoT)
# ★ 敏感信息全部从环境变量读取, 读不到才用占位符. 部署时用 .env 或 export 注入真实值,
#   切勿把真实密钥写进代码提交到公开仓库.
MQTT_BROKER = os.environ.get('MQTT_BROKER', 'your-iot-broker.myhuaweicloud.com')
MQTT_PORT = int(os.environ.get('MQTT_PORT', 8883))
MQTT_DEVICE_ID = os.environ.get('MQTT_DEVICE_ID', 'YOUR_DEVICE_ID')
MQTT_TOPIC = f"$oc/devices/{MQTT_DEVICE_ID}/sys/properties/report"
MQTT_CLIENT_ID = os.environ.get('MQTT_CLIENT_ID', 'YOUR_CLIENT_ID')
MQTT_USERNAME = os.environ.get('MQTT_USERNAME', 'YOUR_USERNAME')
MQTT_PASSWORD = os.environ.get('MQTT_PASSWORD', 'YOUR_PASSWORD')
MQTT_DOWN_TOPIC = f"$oc/devices/{MQTT_DEVICE_ID}/sys/messages/down"

# 串口 — ★ RK3588 上默认 /dev/ttyS9, 支持环境变量覆盖
SERIAL_PORT = os.environ.get('SERIAL_PORT', '/dev/ttyS9')
BAUDRATE = 115200
# 两个 STM32 JSON 包之间最短间隔, 之前 publish 阻塞自带这个延时,
# 现在 publish 异步了, 不留 STM32 解析不过来
SERIAL_CMD_INTERVAL = 0.05

# RTP
RTP_HOST = os.environ.get('RTP_HOST', 'YOUR_RTP_SERVER_IP')
RTP_PORT = int(os.environ.get('RTP_PORT', 5004))

# 摄像头几何 (像素->世界坐标, 与 37.py 一致)
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_HEIGHT_CM = 117
CAMERA_ANGLE_DEG = 36.5
CAMERA_FOV_H = 145.0
CAMERA_FOV_V = 59

# 喷头位置 (相对摄像头)
LEFT_NOZZLE_OFFSET_CM = -26
RIGHT_NOZZLE_OFFSET_CM = 26
NOZZLE_VERTICAL_OFFSET_CM = 67
NOZZLE_DISTANCE_TABLE = [
    (99.2, 111.8, 0),
    (116.4, 127.8, 10),
    (129.0, 145.2, 20),
    (143.8, 157.0, 30),
    (159.0, 170.2, 40),
    (170.4, 183.0, 50),
]

# ============= RKNN 模型 =============
COLOR_RKNN_PATH   = os.path.join(_HERE, 'models', 'best.rknn')      # 彩色 17 类
THERMAL_RKNN_PATH = os.path.join(_HERE, 'models', 'best_ir.rknn')   # 热红外 1 类

# 彩色模型 17 类, 热红外 1 类 (由 rk_convert 从 best.pt/best1.pt 转出来)
COLOR_NUM_CLASSES = 17
COLOR_CLASS_NAMES = {
    0: 'fire', 1: 'smoke ', 2: 'buffalo', 3: 'bear', 4: 'otherentities',
    5: 'Elephant', 6: 'WildBoar', 7: 'cow', 8: 'Tiger', 9: 'Monkey',
    10: 'Deer', 11: 'Leopard', 12: 'goat', 13: 'Raccoon', 14: 'deer',
    15: 'hen', 16: 'gaurd dog',
}
THERMAL_NUM_CLASSES = 1
THERMAL_CLASS_NAMES = {0: 'fire'}

# RK3588 三个 NPU 核全部给彩色 (40.py 已禁用热红外 YOLO)
COLOR_NPU_WORKERS = 3
THERMAL_NPU_WORKERS = 0   # 已废弃, 保留仅供兼容

CONF_THRES_COLOR = 0.25
CONF_THRES_THERMAL = 0.15   # 热红外对比度低, 阈值放宽
IOU_THRES = 0.45

# USB 摄像头 (保留原 USB VID:PID 自动查找, 板子插槽换了不用改代码)
COLOR_CAMERA_USB_VENDOR  = '0c45'
COLOR_CAMERA_USB_PRODUCT = '6368'
THERMAL_CAMERA_USB_VENDOR  = '3474'
THERMAL_CAMERA_USB_PRODUCT = '43d1'
COLOR_CAMERA_FALLBACK_INDEX   = 0
THERMAL_CAMERA_FALLBACK_INDEX = 2
THERMAL_CAMERA_WIDTH  = 384
THERMAL_CAMERA_HEIGHT = 288

# 图像预处理 (抗反光, 与 37.py 一致)
ENABLE_ANTI_GLARE = True
CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_SIZE = 8
BRIGHTNESS_REDUCTION = 0.9

FIRE_DETECTION_CLASSES = {'fire'}
ANIMAL_DETECTION_CLASSES = {
    'buffalo', 'bear', 'otherentities', 'Elephant', 'WildBoar', 'cow',
    'Tiger', 'Monkey', 'Deer', 'Leopard', 'goat', 'Raccoon', 'deer',
    'hen', 'gaurd dog',
}
CLASS_NAME_MAPPING = {
    'fire': '火焰',
    'smoke ': '烟雾', 'smoke': '烟雾',
    'buffalo': '水牛', 'bear': '熊', 'otherentities': '其他实体',
    'Elephant': '大象', 'WildBoar': '野猪', 'cow': '牛',
    'Tiger': '老虎', 'Monkey': '猴子', 'Deer': '鹿',
    'Leopard': '豹子', 'goat': '山羊', 'Raccoon': '浣熊',
    'deer': '鹿', 'hen': '母鸡', 'gaurd dog': '守卫犬',
}

# 应急联动 / 状态机阈值
EMERGENCY_COOLDOWN = 10
SERVO2_EMERGENCY_ANGLE = 45
SWITCH_ON_DURATION = 5

# 滑动窗口比例判断 (与 37.py 一致)
FIRE_DETECTION_WINDOW = 10
FIRE_RATIO_STOP_PATROL = 0.5
FIRE_RATIO_CONFIRM = 0.6
FIRE_RATIO_FALSE_ALARM = 0.2
FIRE_RATIO_EXTINGUISHED = 0.2
FIRE_RESET_WINDOW = 5
CONSECUTIVE_CHECKS = 10

ENABLE_NOZZLE_SWING = True
SWING_ANGLE_RANGE = 10
SWING_DURATION = 1.0
SWING_STEPS = 5

# 视频流带宽 (原项目 low 模式)
JPEG_QUALITY = 30
VIDEO_SCALE_FACTOR = 0.6
RTP_TARGET_FPS = 15         # 上传帧率上限
FRAME_INTERVAL = 1.0 / RTP_TARGET_FPS
RTP_SOCKET_TIMEOUT = 1.0
RTP_MAX_FAILURES = 5

# ★ 采集端节流 (借鉴 x4.py). 摄像头 cap.read() 照做以清 V4L2 缓冲,
# 但只在超时后才把 blob 送 NPU 池, 避免 pool 无限堆积导致内存暴涨/卡死.
# 三核全给彩色后 NPU 消费能力约 40+ FPS, 但 USB 双摄共存下摄像头本身
# 上限就是 13-15 FPS, 所以设 30 只是"不限制"; 实际 FPS 由 read() 阻塞决定.
CAMERA_TARGET_FPS = 30

# ★ 本地预览窗口 (借鉴 x4.py). 无桌面 (DISPLAY 未设置) 自动跳过.
ENABLE_LOCAL_PREVIEW = True
PREVIEW_WINDOW_NAME = "senlin_fanghuo"

# 摄像头重试 (视觉主循环用)
CAMERA_MAX_RETRY = 10
CAMERA_RETRY_THRESHOLD = 5
CAMERA_RETRY_DELAY = 0.5
CAMERA_REINIT_DELAY = 2.0

# 上报节流: 状态变化立即, 同值最长这么久发一次
REPORT_INTERVAL = 2
MQTT_PUBLISH_MIN_INTERVAL = 1.0
LOG_MIN_INTERVAL = 1.0

# 丝杆速度
LEAD_SCREW_MOVE_TIME_PER_10CM = 6
# 丝杆到位判定额外缓冲时间(秒): 在纯软件计时基础上多等这么久再判到位,
# 给下位机阻塞运动留余量, 避免软件早于实际到位就进入下一步.
# 上升和下降复位分开: 上升 +2s, 下降复位 +1s.
LEAD_SCREW_EXTRA_WAIT = 2.0          # 上升
LEAD_SCREW_EXTRA_WAIT_RESET = 1.0    # 下降复位

# 喷头角度变化阈值 (低于就不重下发)
ANGLE_CHANGE_THRESHOLD = 5

# NO_FIRE_RELAY_OFF_THRESHOLD 已被比例判据替代, 保留是为了兼容 37.py 里其它引用
NO_FIRE_RELAY_OFF_THRESHOLD = 5

# ============= 全局变量 =============
running = True
mqtt_client_instance: Optional[mqtt_client.Client] = None
ser: Optional[serial.Serial] = None
color_cap: Optional[cv2.VideoCapture] = None
thermal_cap: Optional[cv2.VideoCapture] = None
color_pool: Optional[RKNNPool] = None
thermal_pool: Optional[RKNNPool] = None
rtp_sender = None
thermal_frame_buffer = None

# 检测状态
current_temp = None
current_humi = None
current_yewei = None
current_auto = 1
fire_status = 0
zoo_status = ""
switch_status = 0
switch1_status = 0
servo1_angle = 90
last_emergency_time = 0
emergency_lock = threading.Lock()
servo1_auto_rotation = True
emergency_in_progress = False
servo1_stop_confirmed = True
servo1_stop_command_time = 0
servo1_stop_retry_count = 0

fire_detection_buffer: list[bool] = []
animal_detection_buffer: list[bool] = []
current_servo2_position = 0

lead_screw_is_up = False
lead_screw_moving = False
lead_screw_move_start_time = 0
lead_screw_actual_move_duration = 0
lead_screw_last_height = 0
no_fire_counter = 0
switch_left_active = False
switch_right_active = False

swing_active = False
swing_start_time = 0
swing_direction = 1
swing_center_angle_servo3 = 90
swing_center_angle_servo4 = 90

FIRE_FIGHTING_STATE = "IDLE"
locked_nozzle: Optional[str] = None
last_servo3_angle: Optional[float] = None
last_servo4_angle: Optional[float] = None

last_fire_report_time = 0
last_zoo_report_time = 0

fly_gpio_available = False
fly_serial = None
fly_current_state = None

# 队列 (都是 size=1, 满了就丢. RK3588 移植的关键)
detection_queue: "queue.Queue" = queue.Queue(maxsize=1)   # consume -> fire_control
rtp_frame_queue: "queue.Queue" = queue.Queue(maxsize=1)   # consume -> rtp_sender
display_frame_queue: "queue.Queue" = queue.Queue(maxsize=1)   # consume -> main thread imshow
color_result_queue: "queue.Queue" = queue.Queue(maxsize=2)   # capture 用来接 pool.get 结果 (用 pool 自带 FIFO)
thermal_annotated_slot: dict = {"frame": None, "fire": False, "lock": threading.Lock()}

# MQTT 上报节流状态
_last_publish = {
    "isfire_value": None, "isfire_ts": 0.0,
    "zoo_value": None,   "zoo_ts": 0.0,
    "switch_value": None, "switch_ts": 0.0,
    "switch1_value": None, "switch1_ts": 0.0,
}

# 日志节流
_log_throttle: dict = {}


def _throttle_log(key, payload, force=False):
    """相同 payload 至少 LOG_MIN_INTERVAL 秒才重复打, 变化立即打."""
    now_ts = time.time()
    rec = _log_throttle.setdefault(key, {"payload": None, "ts": 0.0})
    if force or payload != rec["payload"] or (now_ts - rec["ts"] >= LOG_MIN_INTERVAL):
        rec["payload"] = payload
        rec["ts"] = now_ts
        return True
    return False

# ============= 图像预处理 =============
_clahe_reusable = None   # CLAHE 对象缓存, 不必每帧新建


def preprocess_frame_anti_glare(frame):
    """LAB 空间 CLAHE + 降亮度, 抑制地面反光对火焰检测的干扰."""
    global _clahe_reusable
    try:
        if not ENABLE_ANTI_GLARE:
            return frame
        if _clahe_reusable is None:
            _clahe_reusable = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT,
                                              tileGridSize=(CLAHE_TILE_SIZE, CLAHE_TILE_SIZE))
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = _clahe_reusable.apply(l)
        l = np.clip(l * BRIGHTNESS_REDUCTION, 0, 255).astype(np.uint8)
        return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    except Exception as e:
        _real_print(f"抗反光预处理异常: {e}")
        return frame

# ============= 世界坐标 / 喷头角度计算 =============
def pixel_to_world_coordinate(pixel_x, pixel_y, camera_height=CAMERA_HEIGHT_CM,
                              camera_angle=CAMERA_ANGLE_DEG,
                              img_width=CAMERA_WIDTH, img_height=CAMERA_HEIGHT,
                              servo1_angle=90):
    """像素坐标 -> 地面世界坐标 (cm). 与 37.py 一致."""
    import math
    angle_rad = math.radians(camera_angle)
    fov_h_rad = math.radians(CAMERA_FOV_H)
    fov_v_rad = math.radians(CAMERA_FOV_V)
    cx = img_width / 2.0
    cy = img_height / 2.0
    dx = (pixel_x - cx) / (img_width / 2.0)
    dy = (pixel_y - cy) / (img_height / 2.0)
    theta_h = dx * (fov_h_rad / 2.0)
    theta_v = angle_rad + dy * (fov_v_rad / 2.0)
    if theta_v <= 0 or theta_v >= math.pi / 2:
        return None, None
    distance_forward = camera_height / math.tan(theta_v)
    distance_horizontal = distance_forward * math.tan(theta_h)
    local_x = distance_horizontal
    local_y = distance_forward
    rotation_angle = math.radians(90 - servo1_angle)
    world_x = local_x * math.cos(rotation_angle) - local_y * math.sin(rotation_angle)
    world_y = local_x * math.sin(rotation_angle) + local_y * math.cos(rotation_angle)
    return world_x, world_y


def calculate_nozzle_angles(fire_world_x, fire_world_y, servo1_angle, lead_screw_height=0):
    """根据火焰世界坐标 + 丝杆高度, 计算左/右喷头目标角度."""
    import math
    try:
        if fire_world_x is None or fire_world_y is None:
            return None, None
        if fire_world_y <= 0:
            return None, None
        vertical_distance = max(0, NOZZLE_VERTICAL_OFFSET_CM - lead_screw_height)

        # servo4 = 左, 固定在 x=-26
        dx_l = fire_world_x - LEFT_NOZZLE_OFFSET_CM
        dy_l = fire_world_y
        angle_l = math.degrees(math.atan2(dx_l, dy_l))
        servo4_angle = max(0, min(180, 90 - angle_l))

        # servo3 = 右, 固定在 x=+26
        dx_r = fire_world_x - RIGHT_NOZZLE_OFFSET_CM
        dy_r = fire_world_y
        angle_r = math.degrees(math.atan2(dx_r, dy_r))
        servo3_angle = max(0, min(180, 90 - angle_r))

        _ = vertical_distance   # 保留供俯仰角计算使用
        return servo4_angle, servo3_angle
    except Exception as e:
        _real_print(f"计算喷头角度异常: {e}")
        return None, None


def calculate_lead_screw_height(fire_distance_cm):
    """根据火焰距离查表, 返回丝杆应该升到的高度 (cm)."""
    try:
        if fire_distance_cm is None or fire_distance_cm <= 0:
            return None
        for pump1, pump2, height in NOZZLE_DISTANCE_TABLE:
            if fire_distance_cm <= max(pump1, pump2):
                return height
        return NOZZLE_DISTANCE_TABLE[-1][2]
    except Exception as e:
        _real_print(f"计算丝杆高度异常: {e}")
        return None

# ============= STM32 串口控制 (下发) =============
_serial_write_lock = threading.Lock()


def _stm32_send(data: dict) -> bool:
    """线程安全的串口 JSON 下发. 从任何线程调都可以."""
    if ser is None or not ser.is_open:
        return False
    payload = json.dumps(data, separators=(',', ':')) + '\n'
    with _serial_write_lock:
        try:
            ser.write(payload.encode('utf-8'))
            return True
        except Exception as e:
            _real_print(f"串口写入异常: {e}")
            return False


def control_lead_screw_height(target_height_cm):
    """控丝杆升到指定高度. 返回预计移动时长(秒), 失败返回 None."""
    global lead_screw_is_up, lead_screw_last_height
    try:
        if target_height_cm is None:
            return None
        stm32_distance = int(round(target_height_cm * 10 / 10.0) * 10)
        stm32_distance = max(0, min(500, stm32_distance))
        target_height_cm = stm32_distance / 10.0
        move_duration = (target_height_cm / 10.0) * LEAD_SCREW_MOVE_TIME_PER_10CM + LEAD_SCREW_EXTRA_WAIT

        if not _stm32_send({"motor": "shang", "distance": stm32_distance}):
            return None
        _real_print(f"⬆️  丝杆升至 {target_height_cm}cm (distance={stm32_distance}, 预计{move_duration:.1f}秒)")

        if target_height_cm > 0:
            lead_screw_is_up = True
            lead_screw_last_height = target_height_cm

        if mqtt_client_instance:
            _publish_service({"shang": stm32_distance, "xia": 0})
        return move_duration
    except Exception as e:
        _real_print(f"控制丝杆高度异常: {e}")
        return None


def reset_lead_screw():
    """丝杆复位: 按上次上升高度 + 5cm 下降 (确保完全归零)."""
    global no_fire_counter, lead_screw_moving, lead_screw_move_start_time
    global lead_screw_actual_move_duration
    try:
        descent_cm = lead_screw_last_height + 5
        stm32_distance = int(round(descent_cm * 10 / 10.0) * 10)
        stm32_distance = max(0, min(500, stm32_distance))
        descent_cm = stm32_distance / 10.0

        if not _stm32_send({"motor": "xia", "distance": stm32_distance}):
            _real_print(f"❌ 丝杆复位串口下发失败 (distance={stm32_distance}), 复位未执行")
            return
        _real_print(f"⬇️  丝杆复位 下降{descent_cm}cm (上升{lead_screw_last_height}cm + 5cm, distance={stm32_distance})")

        lead_screw_moving = True
        lead_screw_move_start_time = time.time()
        lead_screw_actual_move_duration = (descent_cm / 10.0) * LEAD_SCREW_MOVE_TIME_PER_10CM + LEAD_SCREW_EXTRA_WAIT_RESET
        no_fire_counter = 0

        if mqtt_client_instance:
            _publish_service({"xia": stm32_distance, "shang": 0})
    except Exception as e:
        _real_print(f"丝杆复位异常: {e}")


def control_single_nozzle_angle(servo_id, angle, silent=False):
    """控制单个喷头 (servo3=右 或 servo4=左) 旋转到指定角度."""
    try:
        if angle is None:
            return
        angle = max(0, min(180, int(round(angle))))
        if not _stm32_send({servo_id: angle}):
            return
        if not silent:
            name = "右喷头" if servo_id == "servo3" else "左喷头"
            _real_print(f"🎯 {name}({servo_id}) -> {angle}°")
        if mqtt_client_instance and not silent:
            _publish_service({servo_id: angle})
    except Exception as e:
        _real_print(f"控制喷头角度异常: {e}")


def control_nozzle_angles(servo4_angle, servo3_angle):
    """左右喷头同时下发."""
    try:
        if servo4_angle is None or servo3_angle is None:
            return
        s4 = max(0, min(180, int(round(servo4_angle))))
        s3 = max(0, min(180, int(round(servo3_angle))))
        if not _stm32_send({"servo4": s4, "servo3": s3}):
            return
        _real_print(f"🎯 喷头 servo4={s4}° servo3={s3}°")
        if mqtt_client_instance:
            _publish_service({"servo4": s4, "servo3": s3})
    except Exception as e:
        _real_print(f"控制喷头角度异常: {e}")


def swing_nozzle():
    """SPRAY 状态下的正弦摇摆, 扩大灭火覆盖."""
    global swing_active, swing_start_time, swing_direction
    global swing_center_angle_servo3, swing_center_angle_servo4
    import math
    try:
        if not ENABLE_NOZZLE_SWING:
            return
        now = time.time()
        if not swing_active:
            swing_active = True
            swing_start_time = now
            swing_direction = 1
            swing_center_angle_servo3 = last_servo3_angle if last_servo3_angle else 90
            swing_center_angle_servo4 = last_servo4_angle if last_servo4_angle else 90
            _real_print(f"🌊 启动喷头摇摆 ±{SWING_ANGLE_RANGE}°")
            return
        elapsed = now - swing_start_time
        progress = (elapsed % SWING_DURATION) / SWING_DURATION
        offset = SWING_ANGLE_RANGE * math.sin(progress * 2 * math.pi)
        if locked_nozzle in ("left", "both"):
            control_single_nozzle_angle("servo4", swing_center_angle_servo4 + offset, silent=True)
        if locked_nozzle in ("right", "both"):
            control_single_nozzle_angle("servo3", swing_center_angle_servo3 + offset, silent=True)
    except Exception as e:
        _real_print(f"喷头摇摆异常: {e}")
        swing_active = False


def control_relay(switch_id, state):
    """继电器控制. switch=右喷头, switch1=左喷头."""
    global switch_left_active, switch_right_active
    try:
        if not _stm32_send({switch_id: state}):
            return
        name = "右喷头继电器(switch)" if switch_id == "switch" else "左喷头继电器(switch1)"
        _real_print(f"💧 {name} {'开启' if state == 1 else '关闭'}")
        if switch_id == "switch":
            switch_right_active = (state == 1)
            publish_switch_status(mqtt_client_instance, state)
        else:
            switch_left_active = (state == 1)
            publish_switch1_status(mqtt_client_instance, state)
    except Exception as e:
        _real_print(f"控制继电器异常: {e}")


def control_voice_alarm(state):
    """语音报警. state=1 开, 0 关."""
    try:
        if _stm32_send({"yuyin": state}):
            _real_print(f"🔊 语音报警 {'开启' if state == 1 else '关闭'}")
    except Exception as e:
        _real_print(f"控制语音报警异常: {e}")


def control_servo1_rotation(enable_rotation):
    """servo1 底座旋转开关. True=巡检旋转, False=停止."""
    global servo1_auto_rotation, servo1_stop_confirmed, servo1_stop_command_time, servo1_stop_retry_count
    try:
        payload = {"servo1stop": 0 if enable_rotation else 1}
        if not _stm32_send(payload):
            return
        _real_print(f"servo1 自动旋转 {'启用' if enable_rotation else '停止'}: {payload}")
        if not enable_rotation:
            servo1_stop_confirmed = False
            servo1_stop_command_time = time.time()
            servo1_stop_retry_count += 1
        else:
            servo1_stop_confirmed = True
            servo1_stop_retry_count = 0
        if mqtt_client_instance:
            _publish_service(payload)
    except Exception as e:
        _real_print(f"控制servo1旋转异常: {e}")


def reset_servo2_position():
    """servo2 归零."""
    try:
        if _stm32_send({"servo2": 0}):
            _real_print("servo2 归零")
            if mqtt_client_instance:
                _publish_service({"servo2": 0})
    except Exception as e:
        _real_print(f"重置servo2位置异常: {e}")


def control_servo2_position(angle):
    """servo2 转到 angle 度."""
    global current_servo2_position
    try:
        if angle == current_servo2_position:
            return
        if not _stm32_send({"servo2": angle}):
            return
        current_servo2_position = angle
        _real_print(f"servo2 -> {angle}°")
        if mqtt_client_instance:
            _publish_service({"servo2": angle})
    except Exception as e:
        _real_print(f"控制servo2位置异常: {e}")


def trigger_emergency():
    """应急联动: 开继电器 -> 5 秒后自动关."""
    global last_emergency_time, emergency_in_progress
    try:
        if emergency_in_progress:
            return
        emergency_in_progress = True
        _stm32_send({"switch": 1})
        last_emergency_time = time.time()
        publish_switch_status(mqtt_client_instance, 1)
        _real_print("自动应急: 继电器开启")

        def _close():
            global emergency_in_progress
            try:
                _stm32_send({"switch": 0})
                publish_switch_status(mqtt_client_instance, 0)
                emergency_in_progress = False
                _real_print("应急开关自动关闭")
            except Exception as e:
                emergency_in_progress = False
                _real_print(f"关闭应急开关异常: {e}")

        t = threading.Timer(SWITCH_ON_DURATION, _close)
        t.daemon = True
        t.start()
    except Exception as e:
        emergency_in_progress = False
        _real_print(f"应急操作异常: {e}")

# ============= MQTT 上报 (异步入队, 业务线程立刻返回) =============
_mqtt_send_queue: "queue.Queue" = queue.Queue(maxsize=256)
_mqtt_dropped = 0


def _publish_service(properties: dict) -> None:
    """把一段 properties 包装成 services 格式, 异步入队."""
    if mqtt_client_instance is None:
        return
    now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    msg = {"services": [{"serviceId": "slmh", "properties": properties, "event_time": now}]}
    tracked_publish(mqtt_client_instance, MQTT_TOPIC, json.dumps(msg))


def tracked_publish(client, topic, payload):
    """异步入队 publish, 立刻返回一个 rc=0 的伪 result. 真实发送在后台线程."""
    global _mqtt_dropped
    class _Fake:
        def __init__(self, rc): self.rc = rc
    if client is None:
        return _Fake(4)
    try:
        _mqtt_send_queue.put_nowait((client, topic, payload))
    except queue.Full:
        _mqtt_dropped += 1
    return _Fake(0)


def mqtt_sender_thread():
    """后台一个个发, 不阻塞业务."""
    while running:
        try:
            client, topic, payload = _mqtt_send_queue.get(timeout=1)
        except queue.Empty:
            continue
        try:
            client.publish(topic, payload)
        except Exception as e:
            _real_print(f"[mqtt_sender] 发送异常: {e}")


def publish_fire_status(client, fire_value):
    """isfire 状态上报, 节流."""
    global fire_status
    try:
        if client is None:
            return
        now = time.time()
        same = (fire_value == _last_publish["isfire_value"])
        if same and (now - _last_publish["isfire_ts"]) < MQTT_PUBLISH_MIN_INTERVAL:
            return
        _last_publish["isfire_value"] = fire_value
        _last_publish["isfire_ts"] = now
        _publish_service({"isfire": bool(fire_value)})
        _real_print(f"✅ isfire 上报: {fire_value}")
    except Exception as e:
        _real_print(f"isfire 上报异常: {e}")


def publish_zoo_status(client, zoo_value):
    """zoo 状态上报, 节流."""
    try:
        if client is None:
            return
        now = time.time()
        same = (zoo_value == _last_publish["zoo_value"])
        if same and (now - _last_publish["zoo_ts"]) < MQTT_PUBLISH_MIN_INTERVAL:
            return
        _last_publish["zoo_value"] = zoo_value
        _last_publish["zoo_ts"] = now
        _publish_service({"zoo": zoo_value})
        _real_print(f"✅ zoo 上报: {zoo_value if zoo_value else '无动物'}")
    except Exception as e:
        _real_print(f"zoo 上报异常: {e}")


def publish_switch_status(client, value):
    global switch_status
    try:
        if client is None:
            return
        switch_status = value
        now = time.time()
        same = (value == _last_publish["switch_value"])
        if same and (now - _last_publish["switch_ts"]) < MQTT_PUBLISH_MIN_INTERVAL:
            return
        _last_publish["switch_value"] = value
        _last_publish["switch_ts"] = now
        _publish_service({"switch": value})
    except Exception as e:
        _real_print(f"switch 上报异常: {e}")


def publish_switch1_status(client, value):
    global switch1_status
    try:
        if client is None:
            return
        switch1_status = value
        now = time.time()
        same = (value == _last_publish["switch1_value"])
        if same and (now - _last_publish["switch1_ts"]) < MQTT_PUBLISH_MIN_INTERVAL:
            return
        _last_publish["switch1_value"] = value
        _last_publish["switch1_ts"] = now
        _publish_service({"switch1": value})
    except Exception as e:
        _real_print(f"switch1 上报异常: {e}")

# ============= 火焰/动物检测处理 + 灭火状态机 =============
def handle_fire_animal_detection(detected_objects, temp, humi, auto_mode, thermal_fire_detected=False):
    """处理火焰/动物检测. detected_objects 每项:
       (class_name, chinese_name, x1, y1, x2, y2, center_x, center_y, [confidence])"""
    global fire_status, zoo_status, servo1_auto_rotation
    global fire_detection_buffer, animal_detection_buffer
    global last_fire_report_time, last_zoo_report_time
    global FIRE_FIGHTING_STATE, locked_nozzle, last_servo3_angle, last_servo4_angle
    global no_fire_counter, lead_screw_is_up, lead_screw_moving, lead_screw_move_start_time
    global lead_screw_actual_move_duration, servo1_stop_confirmed, servo1_stop_command_time
    global servo1_stop_retry_count, swing_active

    try:
        color_fire_detected = any(obj[0] in FIRE_DETECTION_CLASSES and 'fire' in obj[0]
                                  for obj in detected_objects)
        # ★ 火焰判定策略: 只信彩色摄像头 (与 37.py 的 or 融合逻辑不同).
        # 热红外单独报火 (thermal_fire_detected=True) 不再触发 fire_detected,
        # 因为热像仪对热源过于敏感 (人体/发动机/阳光下的金属都会误报),
        # 单独触发会让状态机在 AIM 状态因彩色无坐标而卡住 -> 反复停/复巡检.
        # 热红外仍会在画面右侧显示 IR:FIRE 标签, 只是不进业务判据.
        fire_detected = color_fire_detected
        animals_detected = [obj for obj in detected_objects if obj[0] in ANIMAL_DETECTION_CLASSES]

        fire_detection_buffer.append(fire_detected)
        if len(fire_detection_buffer) > CONSECUTIVE_CHECKS:
            fire_detection_buffer.pop(0)

        animal_detected = len(animals_detected) > 0
        animal_detection_buffer.append(animal_detected)
        if len(animal_detection_buffer) > CONSECUTIVE_CHECKS:
            animal_detection_buffer.pop(0)

        mode_name = "手动模式" if auto_mode == 1 else "自动模式"

        # === 火焰状态上报 ===
        if len(fire_detection_buffer) >= CONSECUTIVE_CHECKS:
            fire_count = sum(fire_detection_buffer)
            new_fire_status = 1 if fire_count == CONSECUTIVE_CHECKS else 0
            now_ts = time.time()
            if new_fire_status != fire_status:
                _real_print(f"🚨 火焰状态变化: {fire_status} -> {new_fire_status} ({mode_name})")
                publish_fire_status(mqtt_client_instance, new_fire_status)
                last_fire_report_time = now_ts
                fire_status = new_fire_status
            elif now_ts - last_fire_report_time >= REPORT_INTERVAL:
                publish_fire_status(mqtt_client_instance, new_fire_status)
                last_fire_report_time = now_ts
                fire_status = new_fire_status

        # === 动物状态上报 ===
        if len(animal_detection_buffer) >= CONSECUTIVE_CHECKS:
            animal_count = sum(animal_detection_buffer)
            now_ts = time.time()
            if animal_count == CONSECUTIVE_CHECKS and animals_detected:
                latest = animals_detected[0][1]
                if latest != zoo_status:
                    _real_print(f"🚨 动物变化: '{zoo_status}' -> '{latest}'")
                    publish_zoo_status(mqtt_client_instance, latest)
                    last_zoo_report_time = now_ts
                    zoo_status = latest
                elif now_ts - last_zoo_report_time >= REPORT_INTERVAL:
                    publish_zoo_status(mqtt_client_instance, latest)
                    last_zoo_report_time = now_ts
            else:
                if zoo_status != "":
                    _real_print(f"🚨 动物消失")
                    publish_zoo_status(mqtt_client_instance, "")
                    last_zoo_report_time = now_ts
                    zoo_status = ""
                elif now_ts - last_zoo_report_time >= REPORT_INTERVAL:
                    publish_zoo_status(mqtt_client_instance, "")
                    last_zoo_report_time = now_ts

        # === 自动模式下的灭火状态机 ===
        if auto_mode != 0:
            return

        window_size = min(len(fire_detection_buffer), FIRE_DETECTION_WINDOW)
        if window_size > 0:
            recent = fire_detection_buffer[-window_size:]
            fire_ratio = sum(recent) / window_size
        else:
            fire_ratio = 0.0

        stop_patrol_fire = (window_size >= 3 and fire_ratio >= FIRE_RATIO_STOP_PATROL)
        fire_danger = (window_size >= 5 and fire_ratio >= FIRE_RATIO_CONFIRM)
        is_false_alarm = (window_size >= FIRE_DETECTION_WINDOW and fire_ratio <= FIRE_RATIO_FALSE_ALARM)

        if not fire_detected:
            no_fire_counter += 1
        else:
            no_fire_counter = 0

        # ★ 状态机诊断日志: 节流打印, 用来定位"识别到火但 servo/丝杆不动"卡在哪个环节.
        # 关注: 状态是否停在 STOP_PATROL(servo1 没停止确认) 还是 AIM(火点坐标 None / 角度 None
        # / lsh None 导致进不了 LIFT). locked_nozzle 非 None 但状态还在 AIM = 卡死信号.
        _sm_diag = (f"[状态机] 状态={FIRE_FIGHTING_STATE} 火比例={fire_ratio*100:.0f}% "
                    f"停巡={stop_patrol_fire} 确认={fire_danger} servo1停止确认={servo1_stop_confirmed} "
                    f"巡检中={servo1_auto_rotation} locked={locked_nozzle} 丝杆已升={lead_screw_is_up}")
        if _throttle_log("sm_diag", _sm_diag):
            _real_print(_sm_diag)

        # ---- 状态 1: IDLE -> STOP_PATROL ----
        if FIRE_FIGHTING_STATE == "IDLE":
            if stop_patrol_fire and servo1_auto_rotation:
                _real_print(f"\n🚨 IDLE -> STOP_PATROL (火焰比例 {fire_ratio*100:.1f}% >= {FIRE_RATIO_STOP_PATROL*100:.0f}%)")
                control_servo1_rotation(False)
                FIRE_FIGHTING_STATE = "STOP_PATROL"

        # ---- 状态 2: STOP_PATROL -> AIM (或误报回 IDLE) ----
        elif FIRE_FIGHTING_STATE == "STOP_PATROL":
            if is_false_alarm:
                _real_print(f"\n🔄 STOP_PATROL -> IDLE (误报, 比例 {fire_ratio*100:.1f}%)")
                FIRE_FIGHTING_STATE = "IDLE"
                no_fire_counter = 0
                servo1_stop_confirmed = True
                servo1_stop_retry_count = 0
                fire_detection_buffer.clear()
                if not servo1_auto_rotation:
                    control_servo1_rotation(True)
            else:
                if not servo1_stop_confirmed:
                    elapsed = time.time() - servo1_stop_command_time
                    if elapsed > 3.0:
                        if servo1_stop_retry_count < 5:
                            _real_print(f"⚠️  servo1 停止超时({elapsed:.1f}s), 重发 (第{servo1_stop_retry_count}次)")
                            control_servo1_rotation(False)
                        else:
                            _real_print("❌ servo1 停止重试 5 次失败, 强制继续")
                            servo1_stop_confirmed = True
                if servo1_stop_confirmed and fire_danger:
                    _real_print(f"\n🔥 STOP_PATROL -> AIM (比例 {fire_ratio*100:.1f}% >= {FIRE_RATIO_CONFIRM*100:.0f}%)")
                    _real_print(f"📡 当前 servo1 角度: {servo1_angle:.1f}°")
                    if locked_nozzle is not None:
                        locked_nozzle = None
                        last_servo3_angle = None
                        last_servo4_angle = None
                    servo1_stop_retry_count = 0
                    FIRE_FIGHTING_STATE = "AIM"

        # ---- 状态 3: AIM 瞄准+锁定喷头 ----
        if FIRE_FIGHTING_STATE == "AIM":
            with emergency_lock:
                fire_points = []
                for obj in detected_objects:
                    if obj[0] in FIRE_DETECTION_CLASSES and len(obj) >= 8:
                        _, cn, _, _, _, _, cx, cy = obj[:8]
                        wx, wy = pixel_to_world_coordinate(cx, cy, servo1_angle=servo1_angle)
                        if wx is not None and wy is not None:
                            fire_points.append({'name': cn, 'world_x': wx, 'world_y': wy})

                if len(fire_points) == 0:
                    pass  # 等下一帧
                elif len(fire_points) == 1:
                    fire = fire_points[0]
                    s4, s3 = calculate_nozzle_angles(fire['world_x'], fire['world_y'],
                                                     servo1_angle, lead_screw_last_height)
                    if s4 is not None and s3 is not None:
                        if locked_nozzle is None:
                            if abs(s4 - 90) < abs(s3 - 90):
                                locked_nozzle = "left"
                                _real_print(f"🔒 锁定左喷头 servo4  目标 X={fire['world_x']:.1f} Y={fire['world_y']:.1f}")
                                last_servo4_angle = s4
                            else:
                                locked_nozzle = "right"
                                _real_print(f"🔒 锁定右喷头 servo3  目标 X={fire['world_x']:.1f} Y={fire['world_y']:.1f}")
                                last_servo3_angle = s3
                            lsh = calculate_lead_screw_height(fire['world_y'])
                            if lsh is not None:
                                dur = control_lead_screw_height(lsh)
                                if dur is not None:
                                    lead_screw_moving = True
                                    lead_screw_move_start_time = time.time()
                                    lead_screw_actual_move_duration = dur
                                    FIRE_FIGHTING_STATE = "LIFT"
                                    _real_print("\n🔄 AIM -> LIFT")
                        elif FIRE_FIGHTING_STATE == "SPRAY":
                            # 微调
                            s4n, s3n = calculate_nozzle_angles(fire['world_x'], fire['world_y'],
                                                                servo1_angle, lead_screw_last_height)
                            if locked_nozzle == "left" and s4n is not None:
                                diff = abs(s4n - last_servo4_angle) if last_servo4_angle else float('inf')
                                if diff >= ANGLE_CHANGE_THRESHOLD:
                                    control_single_nozzle_angle("servo4", s4n)
                                    last_servo4_angle = s4n
                            elif locked_nozzle == "right" and s3n is not None:
                                diff = abs(s3n - last_servo3_angle) if last_servo3_angle else float('inf')
                                if diff >= ANGLE_CHANGE_THRESHOLD:
                                    control_single_nozzle_angle("servo3", s3n)
                                    last_servo3_angle = s3n
                else:
                    # 多火点: 双喷头模式
                    fire_points.sort(key=lambda f: f['world_x'])
                    left_fire = fire_points[0]
                    right_fire = fire_points[-1]
                    if locked_nozzle is None:
                        locked_nozzle = "both"
                        _real_print(f"🔒 双喷头模式 ({len(fire_points)} 个火点)")
                        max_d = max(left_fire['world_y'], right_fire['world_y'])
                        lsh = calculate_lead_screw_height(max_d)
                        if lsh is not None:
                            dur = control_lead_screw_height(lsh)
                            if dur is not None:
                                lead_screw_moving = True
                                lead_screw_move_start_time = time.time()
                                lead_screw_actual_move_duration = dur
                                FIRE_FIGHTING_STATE = "LIFT"
                                _real_print("\n🔄 AIM -> LIFT (双喷头)")
                    elif FIRE_FIGHTING_STATE == "SPRAY":
                        s4, _ = calculate_nozzle_angles(left_fire['world_x'], left_fire['world_y'],
                                                        servo1_angle, lead_screw_last_height)
                        if s4 is not None:
                            diff = abs(s4 - last_servo4_angle) if last_servo4_angle else float('inf')
                            if diff >= ANGLE_CHANGE_THRESHOLD:
                                control_single_nozzle_angle("servo4", s4)
                                last_servo4_angle = s4
                        _, s3 = calculate_nozzle_angles(right_fire['world_x'], right_fire['world_y'],
                                                        servo1_angle, lead_screw_last_height)
                        if s3 is not None:
                            diff = abs(s3 - last_servo3_angle) if last_servo3_angle else float('inf')
                            if diff >= ANGLE_CHANGE_THRESHOLD:
                                control_single_nozzle_angle("servo3", s3)
                                last_servo3_angle = s3

        # ---- 状态 4: LIFT (等丝杆到位 -> SPRAY) ----
        if FIRE_FIGHTING_STATE == "LIFT":
            if lead_screw_moving:
                elapsed = time.time() - lead_screw_move_start_time
                if elapsed >= lead_screw_actual_move_duration:
                    lead_screw_moving = False
                    _real_print(f"\n✅ 丝杆到位 (耗时 {elapsed:.1f}s)")
                    time.sleep(0.2)   # 机械稳定

                    if fire_detected:
                        current_fp = []
                        for obj in detected_objects:
                            if obj[0] in FIRE_DETECTION_CLASSES and 'fire' in obj[0] and len(obj) >= 8:
                                cx, cy = obj[6], obj[7]
                                wx, wy = pixel_to_world_coordinate(cx, cy, servo1_angle=servo1_angle)
                                if wx is not None and wy is not None:
                                    current_fp.append({'world_x': wx, 'world_y': wy})

                        if len(current_fp) > 0:
                            if locked_nozzle in ("left", "right"):
                                fire = current_fp[0]
                                s4, s3 = calculate_nozzle_angles(fire['world_x'], fire['world_y'],
                                                                  servo1_angle, lead_screw_last_height)
                                if locked_nozzle == "left" and s4 is not None:
                                    _real_print(f"🔄 左喷头(servo4) -> {s4:.1f}° 对准火点")
                                    control_single_nozzle_angle("servo4", s4)
                                    last_servo4_angle = s4
                                elif locked_nozzle == "right" and s3 is not None:
                                    _real_print(f"🔄 右喷头(servo3) -> {s3:.1f}° 对准火点")
                                    control_single_nozzle_angle("servo3", s3)
                                    last_servo3_angle = s3
                            elif locked_nozzle == "both":
                                current_fp.sort(key=lambda f: f['world_x'])
                                lf, rf = current_fp[0], current_fp[-1]
                                s4, _ = calculate_nozzle_angles(lf['world_x'], lf['world_y'],
                                                                servo1_angle, lead_screw_last_height)
                                if s4 is not None:
                                    _real_print(f"🔄 左喷头 -> {s4:.1f}°")
                                    control_single_nozzle_angle("servo4", s4)
                                    last_servo4_angle = s4
                                _, s3 = calculate_nozzle_angles(rf['world_x'], rf['world_y'],
                                                                 servo1_angle, lead_screw_last_height)
                                if s3 is not None:
                                    _real_print(f"🔄 右喷头 -> {s3:.1f}°")
                                    control_single_nozzle_angle("servo3", s3)
                                    last_servo3_angle = s3

                            time.sleep(0.3)   # 等喷头到位
                            _real_print("\n🔥 LIFT -> SPRAY")
                            FIRE_FIGHTING_STATE = "SPRAY"
                            control_voice_alarm(1)
                            if locked_nozzle == "left" and not switch_left_active:
                                control_relay("switch1", 1)
                            elif locked_nozzle == "right" and not switch_right_active:
                                control_relay("switch", 1)
                            elif locked_nozzle == "both":
                                if not switch_left_active:
                                    control_relay("switch1", 1)
                                if not switch_right_active:
                                    control_relay("switch", 1)
                    else:
                        # 丝杆到位但当前帧没火 - 判误报或已灭
                        if is_false_alarm:
                            _real_print(f"⚠️ 火焰比例 {fire_ratio*100:.1f}% <= {FIRE_RATIO_FALSE_ALARM*100:.0f}%, 判误报, 直接复位")
                            with emergency_lock:
                                if fire_status == 1:
                                    publish_fire_status(mqtt_client_instance, 0)
                                    fire_status = 0
                                    last_fire_report_time = time.time()
                                if lead_screw_is_up:
                                    reset_lead_screw()
                                    FIRE_FIGHTING_STATE = "RESETTING"
                                else:
                                    FIRE_FIGHTING_STATE = "IDLE"
                                    locked_nozzle = None
                                    if not servo1_auto_rotation:
                                        control_servo1_rotation(True)

        # ---- 状态 5: SPRAY 喷水灭火 (可选摇摆) ----
        if FIRE_FIGHTING_STATE == "SPRAY" and ENABLE_NOZZLE_SWING:
            swing_nozzle()

        # ---- 灭火成功判定 (较小窗口, 反应更快) ----
        reset_window = min(len(fire_detection_buffer), FIRE_RESET_WINDOW)
        if reset_window > 0:
            reset_ratio = sum(fire_detection_buffer[-reset_window:]) / reset_window
        else:
            reset_ratio = 1.0

        extinguished = (reset_window >= FIRE_RESET_WINDOW and
                        reset_ratio <= FIRE_RATIO_EXTINGUISHED and
                        FIRE_FIGHTING_STATE != "RESETTING")

        # ★ 复位诊断日志: 只在灭火活动期 (非 IDLE, 或有喷水/丝杆已升) 打印, 节流避免刷屏.
        # 用来定位"丝杆不复位"到底卡在哪: extinguished 有没有满足, 触发条件里哪个标志不对.
        if FIRE_FIGHTING_STATE != "IDLE" or switch_left_active or switch_right_active or lead_screw_is_up:
            _diag = (f"[复位诊断] 状态={FIRE_FIGHTING_STATE} reset_win={reset_window} "
                     f"reset_ratio={reset_ratio*100:.0f}% extinguished={extinguished} "
                     f"左喷={switch_left_active} 右喷={switch_right_active} "
                     f"丝杆已升={lead_screw_is_up} 丝杆移动中={lead_screw_moving}")
            if _throttle_log("reset_diag", _diag):
                _real_print(_diag)

        if extinguished and (switch_left_active or switch_right_active or lead_screw_is_up):
            with emergency_lock:
                _real_print(f"\n🔄 {FIRE_FIGHTING_STATE} -> RESETTING (灭火成功, 比例 {reset_ratio*100:.1f}%)")
                swing_active = False
                left_was = switch_left_active
                right_was = switch_right_active
                if fire_status == 1:
                    publish_fire_status(mqtt_client_instance, 0)
                    fire_status = 0
                    last_fire_report_time = time.time()
                control_voice_alarm(0)
                if switch_left_active:
                    control_relay("switch1", 0)
                if switch_right_active:
                    control_relay("switch", 0)
                if left_was and right_was:
                    time.sleep(1.0)
                elif left_was or right_was:
                    time.sleep(0.5)
                if switch_left_active or switch_right_active:
                    _real_print("⚠️  继电器状态不同步, 强制重发")
                    if switch_left_active:
                        control_relay("switch1", 0)
                    if switch_right_active:
                        control_relay("switch", 0)
                    time.sleep(0.5)
                if lead_screw_is_up:
                    reset_lead_screw()
                    FIRE_FIGHTING_STATE = "RESETTING"
                else:
                    _real_print(f"\n✅ {FIRE_FIGHTING_STATE} -> IDLE (丝杆已在底部)")
                    FIRE_FIGHTING_STATE = "IDLE"
                    no_fire_counter = 0
                    locked_nozzle = None
                    last_servo3_angle = None
                    last_servo4_angle = None
                    if not servo1_auto_rotation:
                        control_servo1_rotation(True)

        # ---- 状态 6: RESETTING (等丝杆复位完成) ----
        if FIRE_FIGHTING_STATE == "RESETTING":
            # ★ 隐患兜底: 完成判定原本只在 `if lead_screw_moving:` 内执行. 若进入 RESETTING
            # 时 lead_screw_moving 意外为 False (例如灭火中途云端切过 auto 模式, on_message
            # 会把它清 False; 或 reset_lead_screw 因串口失败提前 return 没置位), 完成判定就
            # 永远不跑, 状态永久卡在 RESETTING, 丝杆再也不会复位/巡检不恢复. 这里补一个
            # not moving 的兜底: 直接当作复位完成收尾, 避免死锁.
            if not lead_screw_moving:
                _real_print("⚠️  [复位诊断] 进入 RESETTING 但 lead_screw_moving=False, 兜底直接收尾")
                lead_screw_is_up = False
                no_fire_counter = 0
                FIRE_FIGHTING_STATE = "IDLE"
                locked_nozzle = None
                last_servo3_angle = None
                last_servo4_angle = None
                if not servo1_auto_rotation:
                    control_servo1_rotation(True)
            elif lead_screw_moving:
                elapsed = time.time() - lead_screw_move_start_time
                if elapsed >= lead_screw_actual_move_duration:
                    _real_print("\n✅ RESETTING -> IDLE (丝杆复位完成)")
                    lead_screw_moving = False
                    lead_screw_is_up = False
                    no_fire_counter = 0
                    FIRE_FIGHTING_STATE = "IDLE"
                    locked_nozzle = None
                    last_servo3_angle = None
                    last_servo4_angle = None
                    # ★ 灭火流程结束后主动恢复云台巡检.
                    # 原逻辑只在 RESETTING 完成后打印"保持停止巡检", 恢复巡检全靠下面
                    # `elif not fire_detected` 兜底. 但那个兜底要求"下一帧恰好无火 + 状态位
                    # 全部正确", 只要灭火刚结束画面有一点火焰残留 (某帧 fire_detected=True),
                    # 或 fire_control 线程时序错开, 巡检就一直停着不恢复.
                    # 这里复位完成即无条件重发一次启用巡检: servo1_auto_rotation 标志是等
                    # STM32 串口回传 servo1stop 才置位的, 若回传丢失标志会漂移, 故不加
                    # `if not servo1_auto_rotation` 守卫; 重发 servo1stop:0 对 STM32 幂等无害.
                    _real_print("🔄 灭火流程完成, 恢复云台自动巡检")
                    control_servo1_rotation(True)
        elif not fire_detected:
            if (not servo1_auto_rotation and not lead_screw_is_up
                    and FIRE_FIGHTING_STATE == "IDLE"):
                _real_print("\n🔄 火情结束, 恢复正常巡检")
                control_servo1_rotation(True)

    except Exception as e:
        _real_print(f"火焰动物检测处理异常: {e}")


def handle_ice_detection(detected_ice, temp, humi, auto_mode):
    """兼容性包装: 老代码/模块反射调用时用. 语义等价于 handle_fire_animal_detection.
    (37.py 里保留了这个死代码, x4.py 也保守搬了过来, 这里跟着一起保留.)"""
    detected_objects = []
    if detected_ice:
        detected_objects.append(('fire', '火焰', 0, 0, 100, 100, 50, 50))
    handle_fire_animal_detection(detected_objects, temp, humi, auto_mode)


def handle_manual_servo_control(servo_data):
    """手动模式下的舵机控制 (被 MQTT 下发和 STM32 反馈复用)."""
    global current_servo2_position, servo1_auto_rotation
    try:
        _real_print(f"手动模式舵机控制: {servo_data}")
        if 'servo1' in servo_data and servo_data.get('_source') == 'cloud':
            if servo1_auto_rotation:
                servo1_auto_rotation = False
                _real_print("手动模式下停止 servo1 自动旋转")
        if 'servo1stop' in servo_data:
            stop = bool(servo_data['servo1stop'])
            servo1_auto_rotation = not stop
        if 'servo2' in servo_data:
            current_servo2_position = servo_data['servo2']
        if 'switch' in servo_data:
            publish_switch_status(mqtt_client_instance, servo_data['switch'])
        if 'switch1' in servo_data:
            publish_switch1_status(mqtt_client_instance, servo_data['switch1'])
    except Exception as e:
        _real_print(f"手动模式舵机控制异常: {e}")

# ============= MQTT 连接与下发处理 =============
def connect_mqtt():
    def on_connect(client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            _real_print("MQTT 连接成功")
            client.subscribe(MQTT_DOWN_TOPIC)
            _real_print(f"订阅下发主题: {MQTT_DOWN_TOPIC}")
        else:
            _real_print(f"MQTT 连接失败: {reason_code}")

    def on_disconnect(client, userdata, flags, reason_code, properties=None):
        _real_print(f"MQTT 断开: {reason_code}")

    try:
        client = mqtt_client.Client(client_id=MQTT_CLIENT_ID,
                                     callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
    except AttributeError:
        client = mqtt_client.Client(client_id=MQTT_CLIENT_ID)
        client.on_connect = lambda c, u, f, rc: (
            _real_print("MQTT 连接成功") if rc == 0 else _real_print(f"MQTT 连接失败: {rc}"),
            c.subscribe(MQTT_DOWN_TOPIC) if rc == 0 else None
        )

    client.reconnect_delay_set(min_delay=1, max_delay=30)
    client.username_pw_set(username=MQTT_USERNAME, password=MQTT_PASSWORD)
    client.tls_set()
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        return client
    except Exception as e:
        _real_print(f"MQTT 连接失败: {e}")
        return None


def on_message(client, userdata, msg):
    """处理云端下发. 支持 servo1..6/auto/switch/switch1/shang/xia/fly."""
    global current_auto, current_servo2_position, switch_status, servo1_angle
    global fire_detection_buffer, animal_detection_buffer
    global FIRE_FIGHTING_STATE, locked_nozzle, last_servo3_angle, last_servo4_angle
    global switch_left_active, switch_right_active, lead_screw_is_up, lead_screw_moving
    global no_fire_counter, swing_active

    try:
        payload = msg.payload.decode('utf-8')
        _real_print(f"收到云端下发: {payload}")
        data = json.loads(payload)

        def extract(d):
            r = {}
            if isinstance(d, dict):
                for k, v in d.items():
                    if k in ['servo1', 'servo2', 'servo3', 'servo4', 'servo5', 'servo6',
                             'auto', 'switch', 'switch1', 'shang', 'xia', 'fly']:
                        r[k] = v
                    elif isinstance(v, dict):
                        r.update(extract(v))
                    elif isinstance(v, list):
                        for item in v:
                            r.update(extract(item))
            return r

        s = extract(data)

        if 'fly' in s:
            fv = int(s['fly'])
            control_fly(fv)
            _publish_service({"fly": fv})

        filtered = {}
        motor_data = None
        report_data = {}

        for k in ('servo1', 'servo2', 'servo3', 'servo4', 'servo5', 'servo6'):
            if k in s:
                filtered[k] = int(s[k])
        if 'auto' in s:
            filtered['auto'] = int(s['auto'])
            old = current_auto
            current_auto = filtered['auto']
            if old != current_auto:
                _real_print(f"模式切换: {old} -> {current_auto}")
                if old == 1 and current_auto == 0:
                    fire_detection_buffer.clear()
                    animal_detection_buffer.clear()
                if old == 0 and current_auto == 1:
                    if switch_left_active:
                        control_relay("switch1", 0)
                    if switch_right_active:
                        control_relay("switch", 0)
                    control_voice_alarm(0)
                    if lead_screw_is_up:
                        reset_lead_screw()
                    FIRE_FIGHTING_STATE = "IDLE"
                    locked_nozzle = None
                    last_servo3_angle = None
                    last_servo4_angle = None
                    no_fire_counter = 0
                    swing_active = False
                    lead_screw_moving = False
                    fire_detection_buffer.clear()
                    animal_detection_buffer.clear()
        if 'switch' in s:
            filtered['switch'] = int(s['switch'])
        if 'switch1' in s:
            filtered['switch1'] = int(s['switch1'])

        if 'xia' in s:
            xv = int(s['xia'])
            motor_data = {"motor": "xia", "distance": xv}
            report_data['xia'] = xv
            report_data['shang'] = 0
        if 'shang' in s:
            sv = int(s['shang'])
            motor_data = {"motor": "shang", "distance": sv}
            report_data['shang'] = sv
            report_data['xia'] = 0

        if motor_data:
            _stm32_send(motor_data)

        if filtered:
            _stm32_send(filtered)
            if 'servo2' in filtered:
                current_servo2_position = filtered['servo2']
            if 'servo1' in filtered:
                servo1_angle = filtered['servo1']
            if current_auto == 1:
                cloud_data = dict(filtered)
                cloud_data['_source'] = 'cloud'
                handle_manual_servo_control(cloud_data)
            if 'switch' in filtered:
                switch_status = filtered['switch']

            # 同步回报到云端
            cloud_props = {}
            for k, v in filtered.items():
                cloud_props[k] = bool(v) if k == 'auto' else v
            _publish_service(cloud_props)

        if report_data:
            _publish_service(report_data)
    except Exception as e:
        _real_print(f"处理下发消息异常: {e}")


# ============= 串口相关 =============
def init_fly_gpio():
    """fly 通过主串口发送, 只需要主串口就绪."""
    global fly_gpio_available, fly_serial
    fly_serial = None
    fly_gpio_available = ser is not None
    if fly_gpio_available:
        _real_print(f"fly 使用主串口: {SERIAL_PORT}")
    else:
        _real_print("fly 初始化失败: 主串口未就绪")
    return fly_gpio_available


def control_fly(state):
    """fly 开关命令通过主串口 JSON 下发."""
    global fly_current_state
    if ser is None:
        _real_print(f"fly 主串口不可用 (state={state})")
        return False
    try:
        _stm32_send({"fly": state})
        fly_current_state = state
        _real_print(f"🚁 fly {'开' if state == 1 else '关'}")
        return True
    except Exception as e:
        _real_print(f"fly 控制异常: {e}")
        return False


def cleanup_fly_gpio():
    global fly_gpio_available, fly_serial
    fly_gpio_available = False
    fly_serial = None


def open_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        _real_print(f"串口 {SERIAL_PORT} 已打开")
        return True
    except Exception as e:
        _real_print(f"串口打开失败: {e}")
        ser = None
        return False


def publish_from_serial(client):
    """串口 -> MQTT 上行. 主要传温湿度和 STM32 状态反馈."""
    global current_temp, current_humi, current_yewei, servo1_angle
    global current_servo2_position, switch_status, switch1_status
    global switch_left_active, switch_right_active
    global last_servo3_angle, last_servo4_angle
    global servo1_stop_confirmed, servo1_auto_rotation

    if client is None or ser is None:
        _real_print("MQTT client 或串口未初始化")
        return

    while running:
        try:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if not line:
                continue
            try:
                data = json.loads(line)
            except Exception:
                _real_print(f"串口非 JSON: {line}")
                continue

            props = None
            if 'temp' in data and 'humi' in data:
                current_temp = data['temp']
                current_humi = data['humi']
                p = {"temp": data['temp'], "humi": data['humi']}
                if 'yewei' in data:
                    current_yewei = data['yewei']
                    p['yewei'] = data['yewei']
                props = p
            elif any(k in data for k in ('servo1', 'servo2', 'servo3', 'servo4', 'servo5', 'servo6',
                                          'auto', 'switch', 'switch1', 'shang', 'xia')):
                p = {}
                for k, v in data.items():
                    if k.startswith('servo'):
                        p[k] = v
                        if k == 'servo2':
                            current_servo2_position = v
                        elif k == 'servo1':
                            servo1_angle = v
                        elif k == 'servo3':
                            last_servo3_angle = v
                        elif k == 'servo4':
                            last_servo4_angle = v
                if 'servo1stop' in data:
                    if data['servo1stop'] == 1:
                        servo1_stop_confirmed = True
                        servo1_auto_rotation = False
                        _real_print("✅ servo1 停止已确认")
                    elif data['servo1stop'] == 0:
                        servo1_auto_rotation = True
                        _real_print("✅ servo1 旋转已确认")
                if 'auto' in data:
                    p['auto'] = bool(data['auto'])
                if 'switch' in data:
                    p['switch'] = data['switch']
                    switch_status = data['switch']
                    switch_right_active = (data['switch'] == 1)
                if 'switch1' in data:
                    p['switch1'] = data['switch1']
                    switch1_status = data['switch1']
                    switch_left_active = (data['switch1'] == 1)
                if 'shang' in data:
                    p['shang'] = data['shang']
                if 'xia' in data:
                    p['xia'] = data['xia']
                if current_auto == 1:
                    handle_manual_servo_control(data)
                props = p
            if props:
                _publish_service(props)
        except Exception as e:
            _real_print(f"串口上报异常: {e}")
            time.sleep(1)

# ============= 摄像头 =============
def find_camera_by_usb_id(vendor_id, product_id):
    """通过 USB VID:PID 查 /dev/videoX. 与 37.py 完全一致."""
    import glob
    try:
        for video_path in sorted(glob.glob('/sys/class/video4linux/video*')):
            dev_name = os.path.basename(video_path)
            try:
                idx = int(dev_name.replace('video', ''))
            except ValueError:
                continue
            check = os.path.realpath(os.path.join(video_path, 'device'))
            for _ in range(5):
                vid_file = os.path.join(check, 'idVendor')
                pid_file = os.path.join(check, 'idProduct')
                if os.path.exists(vid_file) and os.path.exists(pid_file):
                    with open(vid_file) as f:
                        vid = f.read().strip().lower()
                    with open(pid_file) as f:
                        pid = f.read().strip().lower()
                    if vid == vendor_id.lower() and pid == product_id.lower():
                        _real_print(f"找到USB摄像头 {vendor_id}:{product_id} -> /dev/video{idx}")
                        return idx
                    break
                parent = os.path.realpath(os.path.join(check, '..'))
                if parent == check:
                    break
                check = parent
    except Exception as e:
        _real_print(f"USB 摄像头查找异常: {e}")
    return None


def _v4l2_warmup(dev_idx, width, height, mjpg=True):
    """OpenCV set() 在部分 UVC 上失效, 用 v4l2-ctl 强制配置."""
    dev = f"/dev/video{dev_idx}"
    cmds = [
        ["v4l2-ctl", "-d", dev, "--set-ctrl=power_line_frequency=0"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=exposure_auto=3"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=exposure_auto_priority=0"],
    ]
    if mjpg:
        cmds.append(["v4l2-ctl", "-d", dev,
                     f"--set-fmt-video=width={width},height={height},pixelformat=MJPG"])
        # 明确要 30 FPS, 否则默认可能是 15 或者被自动曝光拉低
        cmds.append(["v4l2-ctl", "-d", dev, "--set-parm=30"])
    for c in cmds:
        try:
            subprocess.run(c, check=False, capture_output=True, timeout=2)
        except Exception:
            pass


def init_camera():
    """打开彩色摄像头. 用 USB VID:PID 查设备号."""
    global color_cap
    try:
        if color_cap is not None:
            try:
                color_cap.release()
                time.sleep(0.5)
            except Exception:
                pass

        idx = find_camera_by_usb_id(COLOR_CAMERA_USB_VENDOR, COLOR_CAMERA_USB_PRODUCT)
        if idx is None:
            _real_print(f"⚠️  彩色摄像头 USB ID 未找到, 尝试备用 /dev/video{COLOR_CAMERA_FALLBACK_INDEX}")
            idx = COLOR_CAMERA_FALLBACK_INDEX

        # ★ 关键: 用 v4l2-ctl 在 open 之前一次配好格式, 然后 OpenCV 直接用.
        # 之前 warmup + open + set(FOURCC/W/H/FPS) 多次协商 UVC alt setting,
        # 每次协商都独立向 hub 申请带宽预算, 反复 (先低后高) 会撑爆 hub 预算,
        # 导致后开的热红外拿不到带宽. 一次配好后, OpenCV 打开时只协商一次.
        # ★ 关键: 双摄共存的窍门 —— v4l2-ctl 在 open 之前一次配好格式,
        # OpenCV 打开后不再 set 任何流参数, 避免反复协商 UVC alt setting.
        # 反复协商会向 hub 累加带宽预算, 撑爆预算后热红外就拿不到带宽.
        #
        # 副作用: OpenCV 用它自己的默认 (通常 640x480 YUYV), v4l2-ctl 的 MJPG
        # 预配置只影响 UVC 端点协商, OpenCV 里读到的分辨率仍是默认.
        # 这对功能没影响 (YOLO 会 letterbox 到 640x640).
        _v4l2_warmup(idx, CAMERA_WIDTH, CAMERA_HEIGHT, mjpg=True)
        color_cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not color_cap.isOpened():
            _real_print(f"❌ 彩色摄像头打开失败 (/dev/video{idx})")
            return False

        # ★ 关键修复 (双摄带宽): 彩色默认协商成 640x480 YUYV (~147Mbps), 在 USB2 总线
        # (两摄 + 键鼠都挂在同一个板载 hub) 上把带宽占满, 后开的热红外 opened=True 但
        # read() 永远 False (等时传输拿不到带宽). 实测 v4l2-ctl 预设 MJPG 会被 OpenCV
        # 打开时覆盖回 YUYV (见上方注释), 所以必须在 open 之后显式 set FOURCC=MJPG,
        # 把带宽从 ~147Mbps 压到 ~10Mbps, 给热红外腾出空间.
        # 彩色在 init_thermal_camera 之前初始化, 此处设 MJPG 后热红外才开, 顺序天然安全.
        try:
            color_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            color_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            color_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        except Exception as e:
            _real_print(f"⚠️  彩色设 MJPG 失败 (继续用默认格式): {e}")
        try:
            color_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        ret, _ = color_cap.read()
        if not ret:
            _real_print(f"❌ 彩色摄像头首帧失败 (/dev/video{idx})")
            color_cap.release()
            return False

        fcc = int(color_cap.get(cv2.CAP_PROP_FOURCC))
        fcc_str = ''.join(chr((fcc >> (8 * i)) & 0xFF) for i in range(4))
        _real_print(f"[color] device={idx} "
                    f"size={int(color_cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(color_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
                    f"fourcc={fcc_str}")
        return True
    except Exception as e:
        _real_print(f"❌ 彩色摄像头初始化异常: {e}")
        return False


def init_thermal_camera():
    """打开热红外摄像头 (可选)."""
    global thermal_cap
    try:
        if thermal_cap is not None:
            try:
                thermal_cap.release()
                time.sleep(0.5)
            except Exception:
                pass

        idx = find_camera_by_usb_id(THERMAL_CAMERA_USB_VENDOR, THERMAL_CAMERA_USB_PRODUCT)
        if idx is None:
            _real_print(f"⚠️  热红外 USB ID 未找到, 尝试备用 /dev/video{THERMAL_CAMERA_FALLBACK_INDEX}")
            idx = THERMAL_CAMERA_FALLBACK_INDEX

        # ★ 热红外摄像头 (3474:43d1) 有两个坑:
        #   1. 不能做 v4l2 warmup (不支持 exposure_auto 等控制项, 会 -71 EPROTO)
        #   2. 打开后不能调 cap.set(FRAME_WIDTH/HEIGHT) — OpenCV 会关闭 stream
        #      重新协商 UVC 带宽, 此时彩色已经占了 hub 带宽, 协商会被拒.
        #   解决: 用 v4l2-ctl 在 open 之前把格式固定好, cv2.VideoCapture 打开就用.
        #   (依赖 systemd uvc-fix-bandwidth.service 已经设 quirks=0x80 让 UVC
        #   按实际数据速率而非声明的最高带宽分配.)
        try:
            subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/video{idx}",
                 f"--set-fmt-video=width={THERMAL_CAMERA_WIDTH},height={THERMAL_CAMERA_HEIGHT},pixelformat=YUYV"],
                check=False, capture_output=True, timeout=2)
        except Exception:
            pass
        thermal_cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not thermal_cap.isOpened():
            _real_print(f"❌ 热红外摄像头打开失败 (/dev/video{idx})")
            thermal_cap = None
            return False
        # 只设 BUFFERSIZE, 不动分辨率/fps (v4l2-ctl 已经配好)
        try:
            thermal_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # 热像仪首帧比较慢 (FFC 校准 + UVC 端点协商), 最多重试 5 次
        ret = False
        for attempt in range(5):
            ret, _ = thermal_cap.read()
            if ret:
                break
            time.sleep(0.3)
        if not ret:
            _real_print(f"❌ 热红外首帧失败 (/dev/video{idx}) 已重试 5 次")
            thermal_cap.release()
            thermal_cap = None
            return False
        _real_print(f"[thermal] device={idx} size={THERMAL_CAMERA_WIDTH}x{THERMAL_CAMERA_HEIGHT}")
        return True
    except Exception as e:
        _real_print(f"热红外摄像头初始化失败: {e}")
        thermal_cap = None
        return False


# ============= 热红外后台采集线程 =============
class ThermalFrameBuffer:
    """独立线程持续采热红外帧, 主逻辑随时取最新一帧, 避免 V4L2 缓冲堆积."""

    def __init__(self):
        self._frame = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._cap = None

    def start(self, cap):
        self._cap = cap
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True,
                                          name="thermal_reader")
        self._thread.start()
        _real_print("热红外后台采集线程已启动")

    def _reader(self):
        global thermal_cap
        ffc_logged = False
        fail_count = 0
        reconnect_attempts = 0
        while self._running:
            # 如果 cap 从没起来 (init_thermal_camera 首次失败) 或中途掉线, 尝试重连
            if self._cap is None or not self._cap.isOpened():
                if init_thermal_camera():
                    self._cap = thermal_cap
                    fail_count = 0
                    reconnect_attempts = 0
                    _real_print("📷 热红外摄像头 (重新)接入")
                else:
                    reconnect_attempts += 1
                    # 前几次失败 5s 一试, 之后拉到 30s, 免得占 CPU + 刷屏
                    delay = 5.0 if reconnect_attempts < 3 else 30.0
                    if reconnect_attempts <= 3 or reconnect_attempts % 10 == 0:
                        _real_print(f"📷 热红外重连失败 ({reconnect_attempts} 次), {delay}s 后再试")
                    time.sleep(delay)
                    continue
            try:
                ret, frame = self._cap.read()
                if ret and frame is not None:
                    fail_count = 0
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
                    if gray.std() > 10.0:
                        with self._lock:
                            self._frame = frame
                        if ffc_logged:
                            _real_print("📷 热红外 FFC 校准完成")
                            ffc_logged = False
                    else:
                        if not ffc_logged:
                            _real_print("📷 热红外 FFC 校准中...")
                            ffc_logged = True
                else:
                    fail_count += 1
                    if fail_count >= 30:   # 连续 30 帧读不到, 说明设备已挂, 释放重连
                        _real_print("📷 热红外连续读取失败, 释放 cap 准备重连")
                        try:
                            self._cap.release()
                        except Exception:
                            pass
                        self._cap = None
                        thermal_cap = None
                        fail_count = 0
                    time.sleep(0.05)
            except Exception:
                time.sleep(0.05)

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)


# ============= RTP 发送器 =============
class RTPSender:
    def __init__(self, host=RTP_HOST, port=RTP_PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.dest = (host, port)
        self.sequence = 0
        self.ssrc = 0x12345678
        self.failures = 0
        self._init_socket()

    def _init_socket(self):
        try:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(RTP_SOCKET_TIMEOUT)
            _real_print(f"RTP Socket 就绪 {self.host}:{self.port}")
            return True
        except Exception as e:
            _real_print(f"RTP Socket 初始化失败: {e}")
            return False

    def send_frame(self, frame):
        try:
            if VIDEO_SCALE_FACTOR < 1.0:
                nw = int(frame.shape[1] * VIDEO_SCALE_FACTOR)
                nh = int(frame.shape[0] * VIDEO_SCALE_FACTOR)
                frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
            ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ret:
                return
            header = bytearray(12)
            header[0] = 0x80
            header[1] = 0x60
            struct.pack_into('>H', header, 2, self.sequence % 65536)
            struct.pack_into('>I', header, 4, int(time.time()))
            struct.pack_into('>I', header, 8, self.ssrc)
            max_pkt = 60000
            for i in range(0, len(buf), max_pkt):
                chunk = bytes(buf[i:i + max_pkt])
                try:
                    self.sock.sendto(header + chunk, self.dest)
                except (socket.timeout, socket.error, OSError):
                    self.failures += 1
                    if self.failures >= RTP_MAX_FAILURES:
                        _real_print(f"⚠️ RTP 连续失败 {self.failures} 次, 重新初始化 socket")
                        if self._init_socket():
                            self.failures = 0
                    raise
            self.sequence += 1
            if self.failures > 0:
                self.failures = 0
        except Exception as e:
            if self.failures == 1 or self.failures % 5 == 0:
                _real_print(f"RTP 发送失败 ({self.failures}次): {e}")

    def close(self):
        if self.sock:
            self.sock.close()


# ============= 视觉线程 =============
def capture_loop():
    """采集彩色帧 -> letterbox -> 提交彩色 NPU pool. 只做最轻量的活.

    ★ 借鉴 x4.py: 采集端主动限速到 CAMERA_TARGET_FPS.
    cap.read() 照做以清 V4L2 缓冲区(否则 USB 驱动会 disconnect),
    只是超过帧率上限时不再往 NPU 池灌 blob, 保持 pool backlog 稳定在 0-3,
    避免"推理慢->堆积->内存暴涨->卡死"的正反馈.
    """
    global color_cap
    consecutive_failures = 0
    frame_count = 0
    t0 = time.time()
    last_submit_ts = 0.0
    min_frame_interval = 1.0 / max(1, CAMERA_TARGET_FPS)

    while running:
        try:
            if color_cap is None or not color_cap.isOpened():
                _real_print("⚠️ 彩色摄像头未打开, 尝试重新初始化")
                if not init_camera():
                    consecutive_failures += 1
                    if consecutive_failures >= CAMERA_MAX_RETRY:
                        _real_print("❌ 彩色摄像头多次失败, 采集线程退出")
                        break
                    time.sleep(CAMERA_REINIT_DELAY)
                    continue
                consecutive_failures = 0

            ret, frame = color_cap.read()
            if not ret or frame is None:
                consecutive_failures += 1
                if consecutive_failures >= CAMERA_RETRY_THRESHOLD:
                    _real_print("连续读取失败, 重新初始化摄像头")
                    if init_camera():
                        consecutive_failures = 0
                    else:
                        time.sleep(CAMERA_REINIT_DELAY)
                if consecutive_failures >= CAMERA_MAX_RETRY:
                    _real_print("❌ 彩色摄像头多次读取失败, 线程退出")
                    break
                time.sleep(CAMERA_RETRY_DELAY)
                continue
            consecutive_failures = 0

            # ★ 采集限速: 只在超过帧间隔时才送 NPU. 跳过的帧 cap.read() 已经消费,
            # V4L2 缓冲不堆积, USB 驱动不掉线.
            now_ts = time.time()
            if now_ts - last_submit_ts < min_frame_interval:
                continue
            last_submit_ts = now_ts

            # 抗反光 + letterbox 前处理
            processed = preprocess_frame_anti_glare(frame)
            blob, lb = preprocess(processed, new_size=INPUT_SIZE)
            color_pool.put(blob, meta=(frame, lb))

            frame_count += 1
            if frame_count % 100 == 0:
                dt = time.time() - t0
                _real_print(f"[capture] {frame_count} 帧, ~{frame_count/dt:.1f} FPS, "
                            f"color_pool backlog={color_pool.qsize()}")
        except Exception as e:
            _real_print(f"采集线程异常: {e}")
            time.sleep(0.2)
    _real_print("采集线程退出")


def thermal_infer_loop():
    """独立线程: 拿热红外帧 -> 伪彩色化 -> 存入 slot (只显示, 不做 YOLO).

    ★ 40.py 改动: 不再跑 YOLO 推理, 只做画面渲染供拼接显示.
      - 不占 NPU 核 (省下来给彩色用)
      - 不画框, 不打 IR:FIRE 标签, 因为热红外不参与业务判定 (fire_detected 只信彩色)
      - CLAHE + COLORMAP_JET 保持视觉效果 (冷蓝热红), 跟 37.py 一样

    (原来的 YOLO 检测代码保留在 39.py 里可参考.)
    """
    global thermal_annotated_slot
    _clahe_ir = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    while running:
        try:
            if thermal_frame_buffer is None:
                time.sleep(0.1)
                continue
            frame = thermal_frame_buffer.get_frame()
            if frame is None:
                time.sleep(0.02)
                continue
            frame = cv2.flip(frame, -1)   # 修正热像仪安装方向 (水平+垂直翻转)
            frame_r = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT),
                                  interpolation=cv2.INTER_LINEAR)
            gray = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
            gray = _clahe_ir.apply(gray)
            annotated = cv2.applyColorMap(gray, cv2.COLORMAP_JET)

            # 只加一个 "Thermal" 水印标注这半边是什么, 不再有 IR:FIRE 之类
            cv2.putText(annotated, "Thermal", (5, CAMERA_HEIGHT - 8),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

            with thermal_annotated_slot["lock"]:
                thermal_annotated_slot["frame"] = annotated
                thermal_annotated_slot["fire"] = False   # 不再参与判定, 恒 False

            # 节流: 热红外只做显示, 不需要跑那么快, 10 FPS 完全够看
            time.sleep(0.1)
        except Exception as e:
            _real_print(f"热红外显示异常: {e}")
            time.sleep(0.1)
    _real_print("热红外显示线程退出")


def consume_loop():
    """彩色 NPU pool 结果 -> 后处理 -> 中文标注 -> 拼接 -> 送 rtp + fire_control."""
    frame_count = 0
    t0 = time.time()
    while running:
        try:
            out = color_pool.get(timeout=1.0)
            if out is None:
                continue
            outputs, meta = out
            orig_frame, (scale, pad_x, pad_y) = meta
            raw = outputs[0] if isinstance(outputs, (list, tuple)) else outputs

            boxes_xywh, scores, class_ids = decode_yolov8(raw, CONF_THRES_COLOR, COLOR_NUM_CLASSES)
            keep = nms_per_class(boxes_xywh, scores, class_ids, IOU_THRES)
            if keep:
                boxes_xywh = boxes_xywh[keep]
                scores = scores[keep]
                class_ids = class_ids[keep]
                boxes_xyxy = scale_back(boxes_xywh, scale, pad_x, pad_y)
            else:
                boxes_xyxy = np.zeros((0, 4))
                scores = np.zeros((0,))
                class_ids = np.zeros((0,), dtype=np.int64)

            annotated = orig_frame.copy()
            detected_objects = []
            h_img, w_img = annotated.shape[:2]

            for (x1, y1, x2, y2), sc, cid in zip(boxes_xyxy, scores, class_ids):
                class_name = COLOR_CLASS_NAMES.get(int(cid), str(int(cid)))
                if class_name not in FIRE_DETECTION_CLASSES and class_name not in ANIMAL_DETECTION_CLASSES:
                    continue
                cn = CLASS_NAME_MAPPING.get(class_name, class_name)
                ix1 = max(0, min(int(x1), w_img - 1))
                iy1 = max(0, min(int(y1), h_img - 1))
                ix2 = max(0, min(int(x2), w_img - 1))
                iy2 = max(0, min(int(y2), h_img - 1))
                # ★ 坐标系修复 (对齐 37.py): pixel_to_world_coordinate 默认按
                # CAMERA_WIDTH×CAMERA_HEIGHT (320×240) 归一化像素. 但 40.py 的 init_camera
                # 故意不 set() 分辨率 (避免双摄 UVC 带宽重协商), orig_frame 实际是摄像头
                # 原生分辨率 (通常 640×480). 若直接用原生像素中心, dx/dy 会按错误比例归一化,
                # 世界坐标算错 -> servo3/servo4 转到错误角度 (识别到火却不正确动作).
                # 这里把框中心点缩放回 320×240 坐标系, 让下游状态机的世界坐标换算全部正确.
                sx = CAMERA_WIDTH / float(w_img)
                sy = CAMERA_HEIGHT / float(h_img)
                cx = ((ix1 + ix2) / 2.0) * sx
                cy = ((iy1 + iy2) / 2.0) * sy
                detected_objects.append((class_name, cn, ix1, iy1, ix2, iy2, cx, cy, float(sc)))

                color = (0, 0, 255) if class_name in FIRE_DETECTION_CLASSES else (0, 255, 0)
                cv2.rectangle(annotated, (ix1, iy1), (ix2, iy2), color, 2)

                if class_name in FIRE_DETECTION_CLASSES:
                    wx, wy = pixel_to_world_coordinate(cx, cy, servo1_angle=servo1_angle)
                    if wx is not None and wy is not None:
                        coord_text = f"世界: X={wx:.0f} Y={wy:.0f}"
                        annotated = draw_chinese_text(annotated, coord_text, (ix1, iy2 + 5),
                                                        font_size=12, color=(255, 255, 0),
                                                        bg_color=(0, 0, 0))

                label = f"{cn} {float(sc):.2f}"
                annotated = draw_chinese_text(annotated, label, (ix1, iy1 - 5),
                                                font_size=16, color=(255, 255, 255),
                                                bg_color=color)

            if ENABLE_ANTI_GLARE:
                cv2.putText(annotated, "Anti-Glare:ON", (10, 20),
                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(annotated, "Color", (5, CAMERA_HEIGHT - 8),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

            # ★ 保持与 37.py 输出一致: 彩色 320x240 + 热红外 320x240 -> 拼接 640x240.
            # 服务端接收设备已按 37.py 的规格解码, 不能变.
            # 彩色 UVC 实际打开时可能是 640x480 YUYV (因为 open 后不 set 分辨率
            # 会撑爆 USB 带宽), 但发出去之前统一 resize 回 320x240.
            if annotated.shape[:2] != (CAMERA_HEIGHT, CAMERA_WIDTH):
                annotated = cv2.resize(annotated, (CAMERA_WIDTH, CAMERA_HEIGHT),
                                        interpolation=cv2.INTER_AREA)

            with thermal_annotated_slot["lock"]:
                thermal_annotated = thermal_annotated_slot["frame"]
                thermal_fire = thermal_annotated_slot["fire"]
            if thermal_annotated is not None:
                # 热红外原本就 resize 到 CAMERA_WIDTH x CAMERA_HEIGHT (见 thermal_infer_loop),
                # 这里兜底防止意外尺寸
                if thermal_annotated.shape[:2] != (CAMERA_HEIGHT, CAMERA_WIDTH):
                    thermal_annotated = cv2.resize(thermal_annotated,
                                                     (CAMERA_WIDTH, CAMERA_HEIGHT),
                                                     interpolation=cv2.INTER_LINEAR)
                combined = np.hstack([annotated, thermal_annotated])
            else:
                ph = np.full((CAMERA_HEIGHT, CAMERA_WIDTH, 3), 40, dtype=np.uint8)
                cv2.putText(ph, "IR:N/A", (CAMERA_WIDTH // 2 - 35, CAMERA_HEIGHT // 2),
                             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
                combined = np.hstack([annotated, ph])
                thermal_fire = False

            # 送 RTP (size=1 队列, 满了丢旧的)
            try:
                rtp_frame_queue.put_nowait(combined)
            except queue.Full:
                try:
                    rtp_frame_queue.get_nowait()
                    rtp_frame_queue.put_nowait(combined)
                except (queue.Empty, queue.Full):
                    pass

            # 送本地预览 (size=1 队列, 主线程 display_loop 取)
            if ENABLE_LOCAL_PREVIEW:
                try:
                    display_frame_queue.put_nowait(combined)
                except queue.Full:
                    try:
                        display_frame_queue.get_nowait()
                        display_frame_queue.put_nowait(combined)
                    except (queue.Empty, queue.Full):
                        pass

            # 送 fire_control 状态机
            try:
                detection_queue.put_nowait(
                    (list(detected_objects), current_temp, current_humi, current_auto, thermal_fire)
                )
            except queue.Full:
                try:
                    detection_queue.get_nowait()
                    detection_queue.put_nowait(
                        (list(detected_objects), current_temp, current_humi, current_auto, thermal_fire)
                    )
                except (queue.Empty, queue.Full):
                    pass

            frame_count += 1
            if frame_count % 100 == 0:
                dt = time.time() - t0
                _real_print(f"[consume] {frame_count} 帧, ~{frame_count/dt:.1f} FPS, "
                            f"color_pool backlog={color_pool.qsize()}")
        except Exception as e:
            _real_print(f"消费线程异常: {e}")
            time.sleep(0.1)
    _real_print("消费线程退出")


def rtp_sender_thread():
    """RTP 发送线程. 节流到 RTP_TARGET_FPS."""
    min_interval = 1.0 / max(1, RTP_TARGET_FPS)
    last = 0.0
    while running:
        try:
            frame = rtp_frame_queue.get(timeout=1)
            now = time.time()
            if now - last < min_interval:
                continue
            last = now
            if rtp_sender is not None:
                rtp_sender.send_frame(frame)
        except queue.Empty:
            continue
        except Exception as e:
            _real_print(f"rtp_sender 异常: {e}")


def fire_control_thread():
    """消费 detection_queue, 执行灭火状态机 (包含 time.sleep)."""
    while running:
        try:
            args = detection_queue.get(timeout=0.1)
            handle_fire_animal_detection(*args)
        except queue.Empty:
            continue
        except Exception as e:
            _real_print(f"[fire_control] 异常: {e}")


# ============= 本地预览窗口 (主线程) =============
def _have_display() -> bool:
    """检测是否有图形环境. sudo 启动时 DISPLAY/XAUTHORITY 可能丢失, 需要兜底."""
    if not ENABLE_LOCAL_PREVIEW:
        return False
    if not os.environ.get('DISPLAY'):
        return False
    # sudo 情况下继承 xauth
    if os.environ.get('SUDO_USER') and not os.environ.get('XAUTHORITY'):
        candidate = f"/home/{os.environ['SUDO_USER']}/.Xauthority"
        if os.path.exists(candidate):
            os.environ['XAUTHORITY'] = candidate
    return True


def display_loop():
    """主线程驱动 cv2.imshow. 从 display_frame_queue 拿最新一帧显示.

    按 q 或 ESC 退出整个程序. 无桌面则退化成纯 sleep 循环, 程序照常跑."""
    global running
    if not _have_display():
        _real_print("未检测到图形环境 (DISPLAY 未设置), 跳过本地预览窗口")
        _real_print("   提示: 普通用户运行 python3 40.py 就能弹窗;")
        _real_print("        或 sudo 时加 -E 保留环境: sudo -E python3 40.py")
        while running:
            time.sleep(1)
        return

    try:
        cv2.namedWindow(PREVIEW_WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    except cv2.error as e:
        _real_print(f"打开预览窗口失败 ({e}), 降级为无 GUI 模式")
        while running:
            time.sleep(1)
        return

    _real_print(f"📺 本地预览窗口已打开 ({PREVIEW_WINDOW_NAME}), q/ESC 退出")
    last_frame = None
    while running:
        try:
            frame = display_frame_queue.get(timeout=0.5)
            last_frame = frame
        except queue.Empty:
            frame = last_frame
        if frame is not None:
            try:
                cv2.imshow(PREVIEW_WINDOW_NAME, frame)
            except cv2.error:
                break
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            _real_print("预览窗口收到 q/ESC, 退出程序")
            running = False
            break
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


# ============= 信号处理 =============
def signal_handler(sig, frame):
    global running
    _real_print("\n程序被中断, 正在优雅退出...")
    running = False
    try:
        if mqtt_client_instance:
            mqtt_client_instance.disconnect()
            mqtt_client_instance.loop_stop()
    except Exception:
        pass
    try:
        if ser:
            ser.close()
    except Exception:
        pass
    if color_cap:
        try:
            color_cap.release()
        except Exception:
            pass
    if thermal_cap:
        try:
            thermal_cap.release()
        except Exception:
            pass
    if thermal_frame_buffer is not None:
        thermal_frame_buffer.stop()
    if color_pool:
        color_pool.release()
    if thermal_pool:
        thermal_pool.release()
    if rtp_sender:
        rtp_sender.close()
    cv2.destroyAllWindows()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


# ============= 主入口 =============
def check_cameras_on_startup(retry_wait=5, max_retries=3):
    """启动前预检双摄像头. 彩色必须, 热红外可选."""
    def probe(v, p, fallback):
        idx = find_camera_by_usb_id(v, p) or fallback
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False
        ret, f = cap.read()
        try:
            cap.release()
        except Exception:
            pass
        # ★ release() 后 UVC 内核需要~500ms 才允许下次 open, 尤其是热像仪.
        # 不 sleep 会导致后续 init_thermal_camera() 的 first read 失败.
        time.sleep(0.5)
        return ret and f is not None

    color_ok = thermal_ok = False
    for attempt in range(1, max_retries + 1):
        color_ok = probe(COLOR_CAMERA_USB_VENDOR, COLOR_CAMERA_USB_PRODUCT,
                          COLOR_CAMERA_FALLBACK_INDEX)
        thermal_ok = probe(THERMAL_CAMERA_USB_VENDOR, THERMAL_CAMERA_USB_PRODUCT,
                            THERMAL_CAMERA_FALLBACK_INDEX)
        if color_ok:
            if thermal_ok:
                _real_print("✅ 摄像头就绪: 彩色 + 热红外")
            else:
                _real_print("⚠️  彩色就绪, 热红外缺失, 降级运行")
            return color_ok, thermal_ok
        _real_print(f"❌ 彩色摄像头未就绪 (第 {attempt}/{max_retries} 次), {retry_wait}s 后重试")
        time.sleep(retry_wait)
    return color_ok, thermal_ok


def main():
    global mqtt_client_instance, color_pool, thermal_pool, rtp_sender
    global current_temp, current_humi, current_auto, fire_status, zoo_status
    global fire_detection_buffer, animal_detection_buffer, current_servo2_position
    global servo1_auto_rotation, thermal_frame_buffer
    global last_fire_report_time, last_zoo_report_time

    _real_print("=== 森林防火检测系统 (RK3588) 启动 ===")

    color_ok, thermal_ok = check_cameras_on_startup()
    if not color_ok:
        _real_print("❌ 彩色摄像头缺失, 无法启动")
        return

    # 全局变量初值
    current_temp = None
    current_humi = None
    current_auto = 1
    fire_status = 0
    zoo_status = ""
    fire_detection_buffer = []
    animal_detection_buffer = []
    current_servo2_position = 0
    servo1_auto_rotation = True
    last_fire_report_time = 0
    last_zoo_report_time = 0

    # 1. 加载 RKNN 模型
    _real_print(f"加载彩色 RKNN 模型 {COLOR_RKNN_PATH}")
    try:
        color_pool = RKNNPool(COLOR_RKNN_PATH, num_workers=COLOR_NPU_WORKERS,
                              name="color")
    except FileNotFoundError as e:
        _real_print(str(e))
        return
    except Exception as e:
        _real_print(f"彩色 RKNN 加载失败: {e}")
        return

    # ★ 40.py: 热红外不做 YOLO 推理, 不加载 thermal RKNN. 省内存 + 省 NPU.
    thermal_pool = None
    _real_print("热红外 RKNN 已禁用 (40.py 只让彩色识别, 热红外只做显示)")

    # 2. 摄像头
    _real_print("初始化彩色摄像头")
    if not init_camera():
        _real_print("彩色摄像头初始化失败, 退出")
        return
    # 热红外: 首次失败也照常启动后台缓冲线程, 它内部会自动重连
    # 常见触发情形: USB 供电不足/端口枚举失败时首帧读不出来, 但设备后续可能恢复
    _real_print("初始化热红外摄像头 (即使失败也启动后台重连)")
    init_thermal_camera()   # 尝试一次, 失败没关系
    thermal_frame_buffer = ThermalFrameBuffer()
    thermal_frame_buffer.start(thermal_cap)   # cap=None 时 _reader 会自己重连

    # 3. RTP
    _real_print("初始化 RTP")
    try:
        rtp_sender = RTPSender()
    except Exception as e:
        _real_print(f"RTP 初始化失败: {e}")
        return

    # 4. 串口 (可选)
    _real_print(f"打开串口 {SERIAL_PORT}")
    serial_ok = open_serial()
    if not serial_ok:
        _real_print("⚠️  串口打开失败, STM32 控制功能不可用")

    _real_print("初始化 fly")
    init_fly_gpio()

    # 5. MQTT
    _real_print("连接 MQTT")
    mqtt_client_instance = connect_mqtt()
    if not mqtt_client_instance:
        _real_print("MQTT 连接失败, 退出")
        return
    mqtt_client_instance.on_message = on_message
    mqtt_client_instance.loop_start()
    mqtt_client_instance.subscribe(MQTT_DOWN_TOPIC)
    time.sleep(2)

    _real_print("上报初始状态")
    publish_fire_status(mqtt_client_instance, fire_status)
    time.sleep(0.5)
    publish_zoo_status(mqtt_client_instance, zoo_status)

    # 6. 启动汇总
    items = [
        ("彩色摄像头", color_cap is not None and color_cap.isOpened()),
        ("热红外摄像头", thermal_cap is not None and thermal_cap.isOpened()),
        ("串口(STM32)", ser is not None and ser.is_open),
        ("fly串口", fly_gpio_available),
        ("MQTT云端", mqtt_client_instance is not None and mqtt_client_instance.is_connected()),
        ("RTP视频流", rtp_sender is not None),
        ("彩色RKNN", color_pool is not None),
        ("热红外(仅显示)", thermal_frame_buffer is not None),
    ]
    border = "=" * 40
    _real_print("\n" + border)
    _real_print("  系统启动状态汇总 (RK3588)")
    _real_print(border)
    for n, ok in items:
        _real_print(f"  {n:<14} {'✅ 正常' if ok else '❌ 未连接'}")
    _real_print(border + "\n")

    # 7. 启动线程
    threading.Thread(target=fire_control_thread, daemon=True, name="fire_control").start()
    threading.Thread(target=mqtt_sender_thread, daemon=True, name="mqtt_sender").start()
    threading.Thread(target=capture_loop, daemon=True, name="capture").start()
    threading.Thread(target=consume_loop, daemon=True, name="consume").start()
    threading.Thread(target=rtp_sender_thread, daemon=True, name="rtp_sender").start()
    if thermal_frame_buffer is not None:
        # 40.py 里线程只做伪彩色显示, 不再需要 thermal_pool
        threading.Thread(target=thermal_infer_loop, daemon=True, name="thermal_display").start()
    if serial_ok:
        threading.Thread(target=publish_from_serial, args=(mqtt_client_instance,),
                         daemon=True, name="serial_up").start()
        time.sleep(2)
        _real_print("启动 servo1 自动旋转")
        control_servo1_rotation(True)

    _real_print("=== 系统启动完成, 按 Ctrl+C 退出 (预览窗口按 q/ESC 也可以) ===")

    # ★ 主线程跑预览窗口. cv2.imshow 必须在主线程调用, Linux 上其它地方调会崩.
    # 无桌面环境自动降级成 while sleep.
    try:
        display_loop()
    except KeyboardInterrupt:
        _real_print("收到退出信号")

    _real_print("正在清理资源...")
    globals()['running'] = False
    time.sleep(0.5)
    if mqtt_client_instance:
        mqtt_client_instance.disconnect()
        mqtt_client_instance.loop_stop()
    if ser:
        ser.close()
    cleanup_fly_gpio()
    if color_cap:
        color_cap.release()
    if thermal_cap:
        thermal_cap.release()
    if thermal_frame_buffer is not None:
        thermal_frame_buffer.stop()
    if color_pool:
        color_pool.release()
    if thermal_pool:
        thermal_pool.release()
    if rtp_sender:
        rtp_sender.close()
    cv2.destroyAllWindows()
    _real_print("程序已退出")


if __name__ == '__main__':
    main()






