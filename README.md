# ESP32 蓝牙音频频谱灯

> 基于 ESP32 · Bluetooth A2DP 无线音频接收 · I2S 高保真 DAC 输出 · WS2812 实时频谱阵列
> 作者：资深软硬件工程师 · 创业项目 / 开发板产品推介

---

## 一、项目简介

这是一个集**蓝牙音频接收、D 类功放驱动、WS2812 阵列实时频谱显示**于一体的软硬件一体化项目。手机 / 电脑通过经典蓝牙 A2DP 推送音频，ESP32 解码后在两个通道同时输出：

- **I2S 数字音频** → 外接高保真 DAC（PCM5102A）→ D 类功放 → 喇叭发声；
- **实时 FFT 频谱数据** → 驱动 WS2812（NeoPixel）LED 阵列，形成随音乐跳动的彩色频谱柱状图。

整个系统单芯片完成"收声—分析—发声—显形"，可作为创业产品直接量产，也可作为开发板套件面向极客 / 教学市场。

---

## 二、核心特性

| 特性 | 说明 |
| --- | --- |
| 无线音频 | 蓝牙 4.2 经典蓝牙（BR/EDR）+ A2DP Sink，兼容几乎所有手机 / PC |
| 高保真输出 | I2S 16bit / 44.1kHz，外接 PCM5102A DAC（24bit/384kHz，112dB SNR） |
| 实时频谱 | 基于 ESP-DSP 的 512 点复数 FFT，约 86Hz 频率分辨率 |
| 灯阵驱动 | 新版 RMT TX 驱动 WS2812B，级联可扩展（默认 2 屏 × 32 列 × 8 行 = 512 LED） |
| 视觉算法 | 去直流 → 汉宁窗 → FFT → 分频段 → dB 动态范围 → 余辉平滑 → HSV 渐变着色 |
| 低功耗 | 无音频约 5 分钟自动进入软休眠，断连自动清屏 |
| 可扩展 | 分层可插拔架构，支持换窗函数、调 FFT 点数、类库化封装 |

---

## 三、硬件清单（BOM）

### 3.1 主控与音频

| 物料 | 型号 / 参数 | 说明 |
| --- | --- | --- |
| 主控 | ESP32-WROOM-32 | 双核 LX6，支持 Bluetooth Classic + BLE |
| DAC | PCM5102A | I2S 输入，24bit/384kHz，112dB SNR，硬解音频 |
| 功放 | D 类功放模块（PAM8403 / TPA3110 等） | 接收 DAC 输出，驱动喇叭 |
| 喇叭 | 3W / 4Ω 或 5W / 8Ω | 根据功放选型 |
| LDO | 3 × MIC5219 | 分别为 ESP32 / DAC / 功放提供干净电源 |

### 3.2 LED 阵列

| 物料 | 参数 |
| --- | --- |
| LED | WS2812B / NeoPixel，8 行 × 32 列 / 屏 × 2 屏 |
| 级联 | DOUT 串联，`PHYS_COLS = PANELS * 32` |

---

## 四、接线定义

> 全部引脚集中定义在 `main.cpp` 顶部宏，改动只需改一处。

### 4.1 I2S（连接 DAC PCM5102A）

| ESP32 引脚 | 定义 | PCM5102A 端 |
| --- | --- | --- |
| GPIO 25 | `I2S_SCK`  (BCK) | BCK |
| GPIO 27 | `I2S_WS`   (LRCK) | LCK / LRCK |
| GPIO 26 | `I2S_SDOUT`(DIN)  | DIN |
| 3.3V / GND | 电源 / 地 | VCC / GND |

### 4.2 RMT / LED 阵列

| ESP32 引脚 | 定义 |
| --- | --- |
| GPIO 12 | `LED_PIN`（WS2812 数据输入） |

> ⚠️ 注意：WS2812 工作电压 5V，数据线建议串联 330Ω 并加 0.1µF 去耦；长距离级联时数据线加 74HCT245 电平转换。

---

## 五、软件架构

项目采用**分层可插拔**设计，`main.cpp` 单文件即可运行，逻辑分五层：

```
┌──────────────────────────────────────────────┐
│  应用层  loop() 主循环 / 休眠 / 断连管理      │
├──────────────────────────────────────────────┤
│  显示层  HSV→RGB / 蛇形索引 / 余辉平滑        │
├──────────────────────────────────────────────┤
│  算法层  performFFTAndUpdateLEDs() 五步拆解   │
├──────────────────────────────────────────────┤
│  驱动层  RMT-TX(WS2812) / I2S / A2DP(BT)     │
├──────────────────────────────────────────────┤
│  硬件层  ESP32 + DAC + 功放 + 喇叭 + LED 阵列 │
└──────────────────────────────────────────────┘
```

依赖库：

- `AudioTools`、`BluetoothA2DPSink` — 蓝牙 A2DP Sink
- `ESP_I2S` — I2S 音频输出
- `esp-dsp` — FFT + 窗函数（选用 `dsps_fft2r_fc32`）
- `driver/rmt_tx.h` — 新版 RMT LED 驱动
- `Preferences` — 掉电保存音量

---

## 六、FFT 频谱算法详解（五步拆解）

核心函数 `performFFTAndUpdateLEDs()`，每帧执行一次：

### 步骤 ① 去直流偏移
对 512 个采样点求均值并减去，抑制低频噪声与零点漂移。

### 步骤 ② 加窗 + FFT + 取幅度
- 施加 **汉宁窗（Hann）** 减少频谱泄漏；
- 调用 `dsps_fft2r_fc32()` + `dsps_bit_rev_fc32()` 做 512 点复数 FFT；
- 计算各 bin 幅值 `vReal[k] = sqrt(re² + im²)`。

### 步骤 ③ 分频段求和
将 256 个有效频点均匀分到 `CALC_COLS`（显示列 + 2）个频段，每频段取平均能量。

### 步骤 ④ 静音门限 + 空间平滑 + dB 转换
- 总能量 `< 0.001` 判定为静音，直接返回，保持上一帧画面，避免底噪闪烁；
- 对每个频段做**三邻域均值平滑**；
- 转 dB：`db = 20·log10(mag)`，并动态计算 `minDb / maxDb` 做自适应缩放。

### 步骤 ⑤ 动态着色 + 余辉 + 限幅
- 每帧色相 `globalHueOffset` 自增（1.5°/帧），实现彩虹流动；
- 亮度归一化后按行映射 HSV（饱和度 0.80），转 RGB 写入像素缓冲；
- **上升限幅**：每帧最多升高 2 行，防止频谱突跳；
- **余辉**：`prevHeight = prevHeight*0.65 + curH*0.35`（攻击系数 0.35），形成自然下落拖尾。

最终调用 `rmt_ws2812b_send()` 刷新整条灯带。

### 关键参数速查

| 参数 | 值 | 含义 |
| --- | --- | --- |
| `SAMPLES` | 512 | FFT 点数 |
| `SAMPLING_FREQ` | 44100 Hz | 采样率（I2S） |
| 频率分辨率 | ≈86 Hz | 44100 / 512 |
| `PANELS` | 2 | 级联屏数 |
| `PHYS_COLS × PHYS_ROWS` | 64 × 8 | 物理矩阵 |
| `NUM_LEDS` | 512 | 总灯数 |
| `DELAY_MS` | 15 | 主循环帧间隔 |
| `MAX_RISE` | 2 | 每帧最大上升行 |
| 攻击系数 | 0.35 | 余辉上升权重 |
| `HUE_SPEED` | 1.5 | 色相滚动速度 |
| `SLEEP_TIMEOUT` | 5 分钟 | 软休眠阈值 |

---

## 七、构建与烧录

### 7.1 环境

- **PlatformIO**（推荐）或 Arduino IDE + ESP32 板包（≥ 2.x）
- ESP32 分区方案：默认即可

### 7.2 PlatformIO 配置（`platformio.ini`）

```ini
[env:esp32-s3-devkitc1-n16r8]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip
board = esp32dev
framework = arduino
board_upload.flash_size = 16MB
board_build.filesystem = littlefs
board_build.partitions = partition.csv
monitor_speed = 115200
build_type = debug
build_flags = -DBOARD_HAS_PSRAM
	-mfix-esp32-psram-cache-issue
	-mfix-esp32-psram-cache-strategy=memw
	-DCONFIG_SPIRAM_USE_MALLOC=1
	-DSOC_SDMMC_USE_GPIO_MATRIX=1
	-DVERSION=1.0.1
	-D DEBUG=1
	-D CORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_INFO
	
lib_deps = 
	https://github.com/pschatzmann/arduino-audio-tools.git
	https://github.com/pschatzmann/ESP32-A2DP.git

```

> 说明：`BluetoothA2DPSink` 依赖 `ESP32-A2DP`；`AudioTools` 可选，按需引入；`esp-dsp` 随 ESP-IDF 一并提供。

### 7.3 配对使用

1. 上电后串口打印 `BT ready, connect to 'MyMusic'`；
2. 手机 / PC 搜索蓝牙设备 **"EP01"**（代码中 `a2dp_sink.start("EP01", true)`）；
3. 配对码默认 **1234**；
4. 连接成功后播放音乐，LED 阵列即实时显示频谱，同时 DAC/功放输出声音。

---

## 八、许可证

MIT License（可根据创业产品需要调整）。

---

---

---

# ESP32 Bluetooth Audio Spectrum Display 

> Based on ESP32 · Bluetooth A2DP Wireless Audio Reception · I2S Hi-Fi DAC Output · WS2812 Real-Time Spectrum Array
> Author: Senior Hardware & Software Engineer · Startup Project / Dev-Board Product Launch

---

## 1. Introduction

This is an all-in-one hardware/software project that combines **Bluetooth audio reception, Class-D amplifier drive, and a real-time WS2812 spectrum display**. A phone or PC pushes audio over Bluetooth Classic (A2DP); the ESP32 decodes it and simultaneously outputs two channels:

- **I2S digital audio** → external Hi-Fi DAC (PCM5102A) → Class-D amplifier → speaker;
- **Real-time FFT spectrum data** → drives a WS2812 (NeoPixel) LED array, forming a colorful spectrum bar that dances to the music.

A single chip handles "receive → analyze → play → visualize." It can go straight into mass production as a product, or be sold as a dev-board kit for makers and education.

---

## 2. Core Features

| Feature | Description |
| --- | --- |
| Wireless Audio | Bluetooth 4.2 Classic (BR/EDR) + A2DP Sink, compatible with nearly all phones/PCs |
| Hi-Fi Output | I2S 16bit / 44.1kHz, external PCM5102A DAC (24bit/384kHz, 112dB SNR) |
| Real-Time Spectrum | 512-point complex FFT based on ESP-DSP, ~86Hz frequency resolution |
| LED Drive | New RMT TX driver for WS2812B, cascadable (default 2 panels × 32 cols × 8 rows = 512 LEDs) |
| Visual Algorithm | DC removal → Hann window → FFT → band sum → dB dynamic range → afterglow smoothing → HSV gradient |
| Low Power | Auto soft-sleep after ~5 min of no audio; auto clear on disconnect |
| Extensible | Layered pluggable architecture; supports swapping windows, adjusting FFT size, class wrapping |

---

## 3. Bill of Materials (BOM)

### 3.1 Main Control & Audio

| Part | Model / Spec | Description |
| --- | --- | --- |
| MCU | ESP32-WROOM-32 | Dual-core LX6, supports Bluetooth Classic + BLE |
| DAC | PCM5102A | I2S input, 24bit/384kHz, 112dB SNR, hardware audio decode |
| Amplifier | Class-D module (PAM8403 / TPA3110, etc.) | Receives DAC output, drives speaker |
| Speaker | 3W/4Ω or 5W/8Ω | Choose per amplifier |
| LDO | 3 × MIC5219 | Clean power for ESP32 / DAC / amplifier |

### 3.2 LED Array

| Part | Spec |
| --- | --- |
| LED | WS2812B / NeoPixel, 8 rows × 32 cols / panel × 2 panels |
| Cascade | DOUT chained, `PHYS_COLS = PANELS * 32` |

---

## 4. Wiring

> All pins are defined as macros at the top of `main.cpp` — change in one place only.

### 4.1 I2S (to DAC PCM5102A)

| ESP32 Pin | Definition | PCM5102A Side |
| --- | --- | --- |
| GPIO 25 | `I2S_SCK`  (BCK) | BCK |
| GPIO 27 | `I2S_WS`   (LRCK) | LCK / LRCK |
| GPIO 26 | `I2S_SDOUT`(DIN)  | DIN |
| 3.3V / GND | Power / Ground | VCC / GND |

### 4.2 RMT / LED Array

| ESP32 Pin | Definition |
| --- | --- |
| GPIO 12 | `LED_PIN` (WS2812 data input) |

> ⚠️ Note: WS2812 runs at 5V. Add a 330Ω series resistor and 0.1µF decoupling on the data line; use a 74HCT245 level shifter for long cascades.

---

## 5. Software Architecture

The project uses a **layered, pluggable** design. `main.cpp` runs as a single file, with logic split into five layers:

```
┌──────────────────────────────────────────────┐
│ App Layer      loop() main loop / sleep / disconnect │
├──────────────────────────────────────────────┤
│ Display Layer  HSV→RGB / snake indexing / afterglow  │
├──────────────────────────────────────────────┤
│ Algorithm Layer performFFTAndUpdateLEDs() 5-step │
├──────────────────────────────────────────────┤
│ Driver Layer   RMT-TX(WS2812) / I2S / A2DP(BT) │
├──────────────────────────────────────────────┤
│ Hardware Layer ESP32 + DAC + Amp + Speaker + LEDs │
└──────────────────────────────────────────────┘
```

Dependencies:

- `AudioTools`, `BluetoothA2DPSink` — Bluetooth A2DP Sink
- `ESP_I2S` — I2S audio output
- `esp-dsp` — FFT + window functions (uses `dsps_fft2r_fc32`)
- `driver/rmt_tx.h` — New RMT LED driver
- `Preferences` — Persist volume across power cycles

---

## 6. FFT Spectrum Algorithm (5-Step Breakdown)

Core function `performFFTAndUpdateLEDs()` runs once per frame:

### Step ① DC Offset Removal
Compute and subtract the mean of 512 samples to suppress low-frequency noise and zero drift.

### Step ② Window + FFT + Magnitude
- Apply a **Hann window** to reduce spectral leakage;
- Call `dsps_fft2r_fc32()` + `dsps_bit_rev_fc32()` for a 512-point complex FFT;
- Compute each bin magnitude `vReal[k] = sqrt(re² + im²)`.

### Step ③ Band Summation
Map the 256 valid bins evenly into `CALC_COLS` (display columns + 2) bands, averaging energy per band.

### Step ④ Silence Gate + Spatial Smoothing + dB Conversion
- Total energy `< 0.001` → judged silent, return immediately, keep the previous frame to avoid noise flicker;
- Apply **three-neighbor mean smoothing** to each band;
- Convert to dB: `db = 20·log10(mag)`, dynamically compute `minDb / maxDb` for adaptive scaling.

### Step ⑤ Dynamic Coloring + Afterglow + Limiting
- Hue `globalHueOffset` increments each frame (1.5°/frame) for a rainbow flow;
- Normalize brightness, map rows to HSV (saturation 0.80), convert to RGB, write to pixel buffer;
- **Rise limiting**: at most 2 rows per frame to prevent jumping;
- **Afterglow**: `prevHeight = prevHeight*0.65 + curH*0.35` (attack coefficient 0.35) for a natural falling trail.

Finally call `rmt_ws2812b_send()` to refresh the whole strip.

### Key Parameters Quick Reference

| Param | Value | Meaning |
| --- | --- | --- |
| `SAMPLES` | 512 | FFT points |
| `SAMPLING_FREQ` | 44100 Hz | Sampling rate (I2S) |
| Freq Resolution | ≈86 Hz | 44100 / 512 |
| `PANELS` | 2 | Cascaded panel count |
| `PHYS_COLS × PHYS_ROWS` | 64 × 8 | Physical matrix |
| `NUM_LEDS` | 512 | Total LEDs |
| `DELAY_MS` | 15 | Main loop frame interval |
| `MAX_RISE` | 2 | Max rows to rise per frame |
| Attack Coeff | 0.35 | Afterglow rise weight |
| `HUE_SPEED` | 1.5 | Hue scroll speed |
| `SLEEP_TIMEOUT` | 5 min | Soft-sleep threshold |

---

## 7. Build & Flash

### 7.1 Environment

- **PlatformIO** (recommended) or Arduino IDE + ESP32 board package (≥ 2.x)
- ESP32 partition scheme: default

### 7.2 PlatformIO Config (`platformio.ini`)

```ini
[env:esp32-s3-devkitc1-n16r8]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip
board = esp32dev
framework = arduino
board_upload.flash_size = 16MB
board_build.filesystem = littlefs
board_build.partitions = partition.csv
monitor_speed = 115200
build_type = debug
build_flags = -DBOARD_HAS_PSRAM
	-mfix-esp32-psram-cache-issue
	-mfix-esp32-psram-cache-strategy=memw
	-DCONFIG_SPIRAM_USE_MALLOC=1
	-DSOC_SDMMC_USE_GPIO_MATRIX=1
	-DVERSION=1.0.1
	-D DEBUG=1
	-D CORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_INFO
	
lib_deps = 
	https://github.com/pschatzmann/arduino-audio-tools.git
	https://github.com/pschatzmann/ESP32-A2DP.git
```

> Notes: `BluetoothA2DPSink` depends on `ESP32-A2DP`; `AudioTools` is optional; `esp-dsp` ships with ESP-IDF.

### 7.3 Pairing & Usage

1. After power-on, the serial port prints `BT ready, connect to 'MyMusic'`;
2. Search for the Bluetooth device **"EP01"** on your phone/PC (`a2dp_sink.start("EP01", true)`);
3. Default pairing code is **1234**;
4. Once connected, play music — the LED array displays the spectrum in real time while the DAC/amplifier outputs audio.

---

## 8. License

MIT License (adjustable to fit the product's needs).
