
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include "ESP_I2S.h"

/* ---------- esp-dsp：FFT + 窗函数 ---------- */
#include <esp_dsp.h>

/* ---------- 新版 RMT TX 驱动 ---------- */
#include <driver/rmt_tx.h>

// ===================== 级联屏数 =====================
#define PANELS 2
// ===================== 硬件引脚 =====================
const uint8_t I2S_SCK = 25;
const uint8_t I2S_WS = 27;
const uint8_t I2S_SDOUT = 26;
const uint8_t LED_PIN = 12;

// ===================== 物理矩阵 =====================
#define PHYS_BASE_BRIGHTNESS 2
#define PHYS_COLS (PANELS * 32)
#define PHYS_ROWS 8
#define NUM_LEDS (PHYS_COLS * PHYS_ROWS)

// ===================== RMT 像素缓冲 =====================
typedef struct
{
    uint8_t g, r, b;
} pixel_t;
pixel_t pixels[NUM_LEDS];

// ===================== 逻辑显示 =====================
#define DISPLAY_COLS PHYS_COLS
#define CALC_COLS (DISPLAY_COLS + 2)
#define LOGIC_ROWS 8

// ===================== FFT =====================
#define SAMPLES 512
#define SAMPLING_FREQ 44100

double vReal[SAMPLES]; // 幅度输出

// ===================== 音频环缓 =====================
#define AUDIO_BUF_SIZE (SAMPLES * 2)
int16_t audioBuf[AUDIO_BUF_SIZE];
volatile int bufWritePos = 0;
volatile int bufReadPos = 0;

// ===================== 系统对象 =====================
I2SClass i2s;
BluetoothA2DPSink a2dp_sink(i2s);
Preferences prefs;
esp_a2d_connection_state_t last_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;

// ===================== 余辉 =====================
#define DELAY_MS 15
float prevHeight[CALC_COLS] = {0};
const float DECAY_FACTOR = 0.35f;
const float ATTACK_FACTOR = 0.80f;

// ===================== 全局 =====================
unsigned long last_audio_time = 0;
const unsigned long SLEEP_TIMEOUT = 5UL * 60UL * 1000UL;
bool soft_sleep = false;
float globalHueOffset = 0.0f;
const float HUE_SPEED = 1.5f;

// ===================== esp-dsp 工作区 =====================
float fft_data[SAMPLES * 2];
float window[SAMPLES];

// ===================== 新版 RMT 相关 =====================
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t simple_encoder = NULL;

// ===================== 新增全局变量 =====================
// 用于频段能量的时间轴 EMA 平滑
float emaEnergy[CALC_COLS] = {0};    // 初始化为 0
const float ENERGY_EMA_ALPHA = 0.4f; // 平滑系数，越小越平滑（建议 0.3~0.5）

// 用于固定 dB 范围的参考值（通过校准或经验设定）
const double REF_MIN_DB = -70.0; // 最低能量对应的 dB
const double REF_MAX_DB = -20.0; // 最高能量对应的 dB

// 单帧最大高度增量（行数）
const int MAX_HEIGHT_INCREASE = 2; // 每帧最多升高 2 行

// 全局变量：峰值保持数组（在文件开头添加）
float peakHold[CALC_COLS] = {0};
const float PEAK_DECAY = 0.992f; // 每帧衰减0.8%，约1.5秒降至一半

// WS2812 时序常数（基于 10MHz）
static const rmt_symbol_word_t ws2812_zero = {
    .duration0 = (uint16_t)(0.3f * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T0H=0.3us → 3 ticks
    .level0 = 1,
    .duration1 = (uint16_t)(0.9f * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T0L=0.9us → 9 ticks
    .level1 = 0,
};

static const rmt_symbol_word_t ws2812_one = {
    .duration0 = (uint16_t)(0.9f * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T1H=0.9us → 9 ticks
    .level0 = 1,
    .duration1 = (uint16_t)(0.3f * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T1L=0.3us → 3 ticks
    .level1 = 0,
};

static const rmt_symbol_word_t ws2812_reset = {
    .duration0 = (uint16_t)(RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2), // 25us low
    .level0 = 0,
    .duration1 = (uint16_t)(RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2), // 25us low → total 50us
    .level1 = 0,
};

// ===================== 编码器回调 =====================
static size_t encoder_callback(const void *data, size_t data_size,
                               size_t symbols_written, size_t symbols_free,
                               rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    if (symbols_free < 8)
    {
        return 0;
    }

    size_t data_pos = symbols_written / 8;
    uint8_t *data_bytes = (uint8_t *)data;
    if (data_pos < data_size)
    {
        size_t symbol_pos = 0;
        for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1)
        {
            if (data_bytes[data_pos] & bitmask)
            {
                symbols[symbol_pos++] = ws2812_one;
            }
            else
            {
                symbols[symbol_pos++] = ws2812_zero;
            }
        }
        return symbol_pos;
    }
    else
    {
        // 所有字节已编码，发送复位脉冲
        symbols[0] = ws2812_reset;
        *done = 1;
        return 1;
    }
}

// ===================== 新版 RMT 初始化 =====================
void rmt_ws2812b_init()
{
    // 创建 TX 通道（逐个成员赋值，避免设计器顺序问题）
    rmt_tx_channel_config_t tx_chan_config;
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_chan_config.gpio_num = (gpio_num_t)LED_PIN;
    tx_chan_config.mem_block_symbols = 64;
    tx_chan_config.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ;
    tx_chan_config.trans_queue_depth = 4;
    // 其余成员（如 intr_priority、flags）使用默认值 0
    tx_chan_config.intr_priority = 0;
    tx_chan_config.flags.invert_out = 0;
    tx_chan_config.flags.with_dma = 0;
    tx_chan_config.flags.io_loop_back = 0;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    // 创建简单编码器
    const rmt_simple_encoder_config_t simple_encoder_cfg = {
        .callback = encoder_callback};
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_encoder_cfg, &simple_encoder));

    // 使能通道
    ESP_ERROR_CHECK(rmt_enable(led_chan));
}

// ===================== 新版 RMT 发送 =====================
void rmt_ws2812b_send(pixel_t *pix, int len)
{
    // 直接发送像素数据（连续 GRB 字节）
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // 不循环
    };
    ESP_ERROR_CHECK(rmt_transmit(led_chan, simple_encoder, pix, len * 3, &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

// ===================== 蛇形索引 =====================
int getLEDIndex(int physCol, int physRow)
{
    int colFromRight = (PHYS_COLS - 1) - physCol;
    if (colFromRight % 2 == 0)
        return colFromRight * PHYS_ROWS + physRow;
    else
        return colFromRight * PHYS_ROWS + (PHYS_ROWS - 1 - physRow);
}

// ===================== HSV → RGB =====================
void hsv_to_rgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b)
{
    h = fmod(h, 360.0f);
    
    if (h < 0)
        h += 360.0f;
    
    int hi = (int)(h / 60) % 6;
    float f = h / 60.0f - hi;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (hi)
    {
    case 0:
        r = v * 255;
        g = t * 255;
        b = p * 255;
        break;
    case 1:
        r = q * 255;
        g = v * 255;
        b = p * 255;
        break;
    case 2:
        r = p * 255;
        g = v * 255;
        b = t * 255;
        break;
    case 3:
        r = p * 255;
        g = q * 255;
        b = v * 255;
        break;
    case 4:
        r = t * 255;
        g = p * 255;
        b = v * 255;
        break;
    case 5:
        r = v * 255;
        g = p * 255;
        b = q * 255;
        break;
    }
}

// ===================== 蓝牙回调 =====================
void on_data() { last_audio_time = millis(); }
void on_volume_change(int v)
{
    prefs.putUChar("vol", v);
    printf("Vol:%d saved\n", v);
}

void read_data_stream(const uint8_t *data, uint32_t len)
{
    int16_t *s = (int16_t *)data;
    uint32_t frames = len / 4;
    for (uint32_t i = 0; i < frames; i++)
    {
        int16_t mono = (s[i * 2] + s[i * 2 + 1]) / 2;
        int np = (bufWritePos + 1) % AUDIO_BUF_SIZE;
        if (np != bufReadPos)
        {
            audioBuf[bufWritePos] = mono;
            bufWritePos = np;
        }
        else
        {
            bufReadPos = (bufReadPos + 1) % AUDIO_BUF_SIZE;
            audioBuf[bufWritePos] = mono;
            bufWritePos = np;
        }
    }
}

// ===================== FFT + LED =====================
void performFFTAndUpdateLEDs()
{
    int avail = (bufWritePos - bufReadPos + AUDIO_BUF_SIZE) % AUDIO_BUF_SIZE;
    if (avail < SAMPLES)
        return;

    int16_t samp[SAMPLES];
    for (int i = 0; i < SAMPLES; i++)
    {
        samp[i] = audioBuf[bufReadPos];
        bufReadPos = (bufReadPos + 1) % AUDIO_BUF_SIZE;
    }

    // ① 保留直流偏移去除（有助于减少低频噪声）
    int32_t sum = 0;
    for (int i = 0; i < SAMPLES; i++)
        sum += samp[i];
    int16_t mean = sum / SAMPLES;
    for (int i = 0; i < SAMPLES; i++)
        samp[i] -= mean;

    // ② 加窗、FFT、取幅度（完全不变）
    for (int i = 0; i < SAMPLES; i++)
    {
        fft_data[2 * i] = (float)samp[i] * window[i];
        fft_data[2 * i + 1] = 0.0f;
    }
    dsps_fft2r_fc32(fft_data, SAMPLES);
    dsps_bit_rev_fc32(fft_data, SAMPLES);

    for (int k = 0; k < SAMPLES / 2; k++)
    {
        float re = fft_data[2 * k];
        float im = fft_data[2 * k + 1];
        vReal[k] = sqrt((double)(re * re + im * im));
    }

    // ③ 分频段求和（不变）
    double rawEnergy[CALC_COLS] = {0};
    int binsPerBand = (SAMPLES / 2) / CALC_COLS;
    if (binsPerBand < 1)
        binsPerBand = 1;

    for (int band = 0; band < CALC_COLS; band++)
    {
        double sum = 0.0;
        int sb = band * binsPerBand;
        int eb = sb + binsPerBand;
        if (eb > SAMPLES / 2)
            eb = SAMPLES / 2;
        for (int b = sb; b < eb; b++)
            sum += vReal[b];
        rawEnergy[band] = sum / binsPerBand;
    }

    // ④ ★★★ 恢复动态 dB 范围（和你原始代码一样）★★★
    // 但增加一个能量门限：如果所有频段总能量太低，直接返回，保持上一帧画面
    double totalEnergy = 0;
    for (int i = 0; i < CALC_COLS; i++)
        totalEnergy += rawEnergy[i];
    if (totalEnergy < 0.001)
        return; // 静音时不更新，避免噪声闪烁

    // 三倍空间平滑（不变）
    double smoothed[CALC_COLS];
    for (int i = 0; i < CALC_COLS; i++)
    {
        double s = rawEnergy[i];
        int c = 1;
        if (i > 0)
        {
            s += rawEnergy[i - 1];
            c++;
        }
        if (i < CALC_COLS - 1)
        {
            s += rawEnergy[i + 1];
            c++;
        }
        smoothed[i] = s / c;
    }

    // dB 转换 + 动态范围计算（完全恢复原始逻辑）
    double db[CALC_COLS], minDb = 1e9, maxDb = -1e9;
    for (int i = 2; i < CALC_COLS; i++)
    {
        double m = smoothed[i];
        if (m < 1e-6)
            m = 1e-6;
        db[i] = 20.0 * log10(m);
        if (db[i] < minDb)
            minDb = db[i];
        if (db[i] > maxDb)
            maxDb = db[i];
    }
    double range = maxDb - minDb;
    if (range < 1.0)
        range = 1.0;

    // ⑤ ★★★ 唯一的新增：限制每帧上升行数（防突跳）★★★
    const int MAX_RISE = 2; // 每帧最多上升2行（共8行）

    globalHueOffset += HUE_SPEED;
    if (globalHueOffset >= 360.0f)
        globalHueOffset -= 360.0f;

    // ★★★ 恢复原始亮度计算 ★★★
    float bright = PHYS_BASE_BRIGHTNESS / 260.0f; // 2/260≈0.0077，稍亮一点

    for (int ci = 2; ci < CALC_COLS; ci++)
    {
        double norm = (db[ci] - minDb) / range;
        int curH = (int)(norm * LOGIC_ROWS);
        if (curH > LOGIC_ROWS)
            curH = LOGIC_ROWS;
        if (curH < 0)
            curH = 0;

        // 限制上升速度
        int prevH = (int)(prevHeight[ci]);
        if (curH > prevH + MAX_RISE)
            curH = prevH + MAX_RISE;

        // ★★★ 降低攻击系数（原0.8→0.35），衰减系数不变（0.35）★★★
        prevHeight[ci] = prevHeight[ci] * 0.65f + curH * 0.35f;
        int fh = (int)(prevHeight[ci] + 0.5f);
        if (fh > LOGIC_ROWS)
            fh = LOGIC_ROWS;
        if (fh < 0)
            fh = 0;

        int pc = ci - 2;
        for (int lr = 0; lr < LOGIC_ROWS; lr++)
        {
            int li = getLEDIndex(pc, lr);
            if (lr < fh)
            {
                float bh = map(lr, 0, LOGIC_ROWS - 1, 0, 120) + globalHueOffset;
                uint8_t r, g, b;
                hsv_to_rgb(bh, 0.80f, 1.0f, r, g, b);
                pixels[li].r = (uint8_t)(r * bright);
                pixels[li].g = (uint8_t)(g * bright);
                pixels[li].b = (uint8_t)(b * bright);
            }
            else
            {
                pixels[li].r = pixels[li].g = pixels[li].b = 0;
            }
        }
    }

    rmt_ws2812b_send(pixels, NUM_LEDS);
}
// ===================== setup =====================
void setup()
{
    printf("FFT Spectrum + BT A2DP, panels=%d cols=%d\n", PANELS, PHYS_COLS);

    // 初始化新版 RMT
    rmt_ws2812b_init();
    memset(pixels, 0, sizeof(pixels));
    rmt_ws2812b_send(pixels, NUM_LEDS); // 全灭

    // esp-dsp 初始化
    esp_err_t e = dsps_fft2r_init_fc32(NULL, SAMPLES);
    if (e != ESP_OK)
    {
        printf("FFT init fail %d\n", e);
        while (1)
            delay(100);
    }

    // 汉宁窗
    dsps_wind_hann_f32(window, SAMPLES);

    // I2S
    i2s.setPins(I2S_SCK, I2S_WS, I2S_SDOUT);
    if (!i2s.begin(I2S_MODE_STD, SAMPLING_FREQ, I2S_DATA_BIT_WIDTH_16BIT,
                   I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH))
    {
        printf("I2S init fail\n");
        while (1)
            delay(100);
    }

    // BT A2DP
    esp_bluedroid_config_t bcfg = {};
    bcfg.ssp_en = false;
    a2dp_sink.set_bluedroid_config_t(bcfg);
    a2dp_sink.set_on_data_received(on_data);
    a2dp_sink.set_stream_reader(read_data_stream);
    a2dp_sink.set_avrc_rn_volumechange(on_volume_change);

    uint8_t vol = prefs.getUChar("vol", 30);
    a2dp_sink.set_volume(vol);
    a2dp_sink.start("EP01", true);

    esp_bt_io_cap_t ioc = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &ioc, 1);
    esp_bt_pin_type_t pt = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin = {};
    memcpy(pin, "1234", 4);
    esp_bt_gap_set_pin(pt, 4, pin);

    printf("BT ready, connect to 'MyMusic'\n");
}

// ===================== loop =====================
void loop()
{
    auto st = a2dp_sink.get_connection_state();
    if (st == ESP_A2D_CONNECTION_STATE_CONNECTED)
    {
        if (soft_sleep)
        {
            soft_sleep = false;
            printf("Wake\n");
        }
        performFFTAndUpdateLEDs();
        last_audio_time = millis();
    }
    else
    {
        static bool cleared = false;
        if (!cleared)
        {
            memset(pixels, 0, sizeof(pixels));
            rmt_ws2812b_send(pixels, NUM_LEDS);
            printf("BT disconnected, advertising...\n");
            cleared = true;
            delay(1000);
        }
        if (!soft_sleep && millis() - last_audio_time > SLEEP_TIMEOUT)
        {
            soft_sleep = true;
            memset(pixels, 0, sizeof(pixels));
            rmt_ws2812b_send(pixels, NUM_LEDS);
            printf("Sleep\n");
            delay(1000);
        }
        cleared = false;
    }
    if (last_state != st)
    {
        printf(st == ESP_A2D_CONNECTION_STATE_CONNECTED ? "Connected\n" : "Disconnected\n");
        last_state = st;
    }
    delay(DELAY_MS);
}