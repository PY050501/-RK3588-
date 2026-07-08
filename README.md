# -RK3588-

Elf2、Rk3588、物联网、嵌入式

森林防火检测系统:**RK3588 上位机**(视觉识别 + 灭火决策 + 云端上报)+ **STM32F103 下位机**(舵机/步进电机/丝杆/继电器执行)配套工程。

---

## RK3588 上位机 (44.py)

基于 RK3588 NPU 的森林防火实时检测系统:彩色 + 热红外双摄像头,YOLOv8 火焰/动物识别,
自动灭火状态机(舵机瞄准 + 丝杆升降 + 继电器喷水),华为云 IoT 上报,RTP 视频流推送。

### 硬件

- RK3588 开发板(3 个 NPU 核)
- USB 彩色摄像头(17 类模型)
- USB 热红外摄像头(仅显示)
- STM32 下位机(串口 JSON 控制舵机/丝杆/继电器)

### 架构

多线程解耦,队列 size=1(满则丢旧留新):

- `capture_loop` — 采集彩色帧 -> letterbox -> 送 NPU 池(采集端限速防 backlog)
- `consume_loop` — NPU 结果 -> 后处理/NMS -> 中文标注 -> 拼接热红外 -> 分发
- `fire_control_thread` — 灭火状态机 IDLE -> STOP_PATROL -> AIM -> LIFT -> SPRAY -> RESETTING
- `thermal_infer_loop` — 热红外伪彩色显示(不跑 YOLO,省 NPU)
- `rtp_sender_thread` / `mqtt_sender_thread` / `publish_from_serial` — 视频流 / 上报 / 串口上行

### 配置

敏感信息(MQTT 密钥、服务器 IP)全部通过环境变量注入,**代码中不含真实凭据**。

```bash
cp .env.example .env      # 填入真实值
set -a; source .env; set +a
python3 44.py
```

### 依赖

- `rk_runtime/`(RKNNPool、yolov8 后处理、中文绘制)—— 需单独放置
- `models/best.rknn`、`models/best_ir.rknn` —— 模型文件,单独分发
- opencv-python、numpy、pyserial、paho-mqtt、rknnlite

### 已知问题

双摄 USB 带宽:两路 UVC 相机需分属不同 USB 控制器(避免同一个 ehci 上的
isoc 带宽冲突导致第二路 read() 失败)。

---

## STM32F103 下位机 (Keil 工程)

基于 STM32F103ZE + 标准外设库的执行控制器,通过串口接收 RK3588 下发的 JSON 指令:

- 舵机(servo1~6):云台旋转、喷头瞄准
- 步进电机 / 丝杆:喷头升降定位
- 继电器(switch/switch1):左右喷头喷水
- Si7021:温湿度采集上报
- TFT LCD + 触摸:本地显示
- I2C / AT24Cxx:参数存储

用 Keil MDK 打开 `Template.uvprojx` 编译。
