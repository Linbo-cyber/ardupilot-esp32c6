# ArduPilot ESP32-C6 Rover

轻量级 ArduPilot Rover 固件，专为 ESP32-C6 设计。兼容 QGroundControl / Mission Planner。

## 特性

- MAVLink 2.0 协议，QGC 自动识别为 ArduPilot Rover
- USB CDC 串口（主链路）+ WiFi AP UDP（备用链路）
- ADC 电池电压检测 + 低压/危险报警
- PID 转向和油门控制
- 参数系统（NVS 持久化，QGC 可读写）
- RC Override 遥控
- 通信丢失 Failsafe + 电池 Failsafe
- 50Hz 控制环

## 硬件接线

| 功能 | GPIO | 说明 |
|------|------|------|
| 油门 PWM | GPIO4 | 接电调信号线 |
| 转向 PWM | GPIO5 | 接舵机信号线 |
| 电池 ADC | GPIO0 (ADC1_CH0) | 电阻分压：R1=30kΩ → R2=10kΩ → GND |
| USB | 原生 USB | OTG 线连手机 |

### 电池分压电路

```
VBAT ──┤30kΩ├──┬──┤10kΩ├── GND
               │
            GPIO0 (ADC)
```

3S 电池 12.6V → 分压后 3.15V，在 ADC 量程内。

## 构建

```bash
# 安装 ESP-IDF v5.3
# https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32c6/get-started/

# 下载 MAVLink 头文件
./scripts/fetch_mavlink.sh

# 构建
idf.py set-target esp32c6
idf.py build

# 烧录
idf.py flash
```

## 使用

1. USB OTG 线连接 C6 到手机
2. 手机打开 QGroundControl，自动识别 Rover
3. 或连接 WiFi `Rover-C6`（密码 `ardupilot`），QGC 添加 UDP 14550 连接

## WiFi 备用链路

C6 启动后会创建 AP：
- SSID: `Rover-C6`
- 密码: `ardupilot`
- MAVLink UDP 端口: 14550

## 参数

通过 QGC 参数面板可调：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| STEER_P | 1.0 | 转向 P 增益 |
| STEER_I | 0.1 | 转向 I 增益 |
| STEER_D | 0.01 | 转向 D 增益 |
| THR_P | 0.5 | 油门 P 增益 |
| BATT_LOW_V | 10.5 | 低压报警电压 |
| BATT_CRT_V | 9.6 | 危险电压（自动停车） |

## License

MIT
