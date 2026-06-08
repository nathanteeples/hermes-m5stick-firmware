#pragma once

#ifndef HERMES_BOARD_HOSYOND_ES3C28P

#include <M5StickCPlus2.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern SemaphoreHandle_t nvsMutex;
extern SemaphoreHandle_t tamaMutex;


// Typedefs for display compatibility
#define TFT_eSPI LovyanGFX
#define TFT_eSprite LGFX_Sprite

// RTC structures compatibility
struct RTC_TimeTypeDef {
  uint8_t Hours;
  uint8_t Minutes;
  uint8_t Seconds;
};

struct RTC_DateTypeDef {
  uint8_t WeekDay;
  uint8_t Month;
  uint8_t Date;
  uint16_t Year;
};

class RTC_Compat {
public:
  void GetTime(RTC_TimeTypeDef* time) {
    auto t = ::M5.Rtc.getTime();
    time->Hours = t.hours;
    time->Minutes = t.minutes;
    time->Seconds = t.seconds;
  }
  void GetDate(RTC_DateTypeDef* date) {
    auto dt = ::M5.Rtc.getDateTime();
    date->WeekDay = dt.date.weekDay;
    date->Month = dt.date.month;
    date->Date = dt.date.date;
    date->Year = dt.date.year;
  }
  void SetTime(RTC_TimeTypeDef* time) {
    auto dt = ::M5.Rtc.getDateTime();
    ::M5.Rtc.setDateTime( { { dt.date.year, dt.date.month, dt.date.date, dt.date.weekDay },
                            { (int8_t)time->Hours, (int8_t)time->Minutes, (int8_t)time->Seconds } } );
  }
  void SetDate(RTC_DateTypeDef* date) {
    auto dt = ::M5.Rtc.getDateTime();
    ::M5.Rtc.setDateTime( { { (int16_t)date->Year, (int8_t)date->Month, (int8_t)date->Date, (int8_t)date->WeekDay },
                            { dt.time.hours, dt.time.minutes, dt.time.seconds } } );
  }
};

class IMU_Compat {
public:
  int Init() {
    return ::M5.Imu.init();
  }
  bool getAccelData(float* ax, float* ay, float* az) {
    return ::M5.Imu.getAccelData(ax, ay, az);
  }
};

class Beep_Compat {
public:
  void begin() {}
  void update() {}
  void tone(uint16_t freq, uint16_t dur) {
    ::M5.Speaker.tone(freq, dur);
  }
};

class AXP192_Compat {
public:
  void ScreenBreath(uint8_t brightness) {
    uint8_t val = (uint8_t)(brightness * 2.55f);
    if (val < 4) val = 4;  // avoid complete black-out, SetLDO2 handles off
    ::M5.Display.setBrightness(val);
  }
  void SetLDO2(bool state) {
    if (state) {
      // Handled by subsequent ScreenBreath calls
    } else {
      ::M5.Display.setBrightness(0);
    }
  }
  void PowerOff() {
    ::M5.Power.powerOff();
  }
  float GetBatVoltage() {
    return (float)::M5.Power.getBatteryVoltage() / 1000.0f;
  }
  float GetBatCurrent() {
    return (float)::M5.Power.getBatteryCurrent();
  }
  int GetBatLevel() {
    return (int)::M5.Power.getBatteryLevel();
  }
  float GetVBusVoltage() {
    auto t = ::M5.Power.getType();
    if (t == m5::Power_Class::pmic_axp2101) return ::M5.Power.Axp2101.getVBUSVoltage();
    if (t == m5::Power_Class::pmic_axp192)  return ::M5.Power.Axp192.getVBUSVoltage();
    auto chg = ::M5.Power.isCharging();
    if (chg == m5::Power_Class::is_charging) return 5.0f;
    float vbat = (float)::M5.Power.getBatteryVoltage() / 1000.0f;
    return (vbat > 4.1f) ? 5.0f : 0.0f;
  }
  float GetTempInAXP192() {
    auto t = ::M5.Power.getType();
    if (t == m5::Power_Class::pmic_axp2101) return ::M5.Power.Axp2101.getInternalTemperature();
    if (t == m5::Power_Class::pmic_axp192)  return ::M5.Power.Axp192.getInternalTemperature();
    return -999.0f;
  }
  uint8_t GetBtnPress() {
    if (::M5.BtnPWR.wasClicked()) {
      return 0x02;
    }
    return 0;
  }
};

class M5StickCPlus_Compat {
public:
  M5GFX &Lcd = ::M5.Display;
  IMU_Compat Imu;
  RTC_Compat Rtc;
  m5::Button_Class &BtnA = ::M5.BtnA;
  m5::Button_Class &BtnB = ::M5.BtnB;
  m5::Mic_Class     &Mic  = ::M5.Mic;
  
  AXP192_Compat Axp;
  Beep_Compat Beep;

  void begin() {
    Serial.begin(115200);
    delay(10);
    Serial.println("M5StickCPlus_Compat: beginning...");
    auto cfg = ::M5.config();
    ::M5.begin(cfg);
    ::M5.Display.setBrightness(128);
  }

  void update() {
    ::M5.update();
  }
};

inline M5StickCPlus_Compat M5_compat;
#define M5 M5_compat

#else

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <driver/i2c.h>
#include <driver/i2s.h>
#include <esp_sleep.h>
#include <sys/time.h>
#include <time.h>
#include "es8311.h"

extern SemaphoreHandle_t nvsMutex;
extern SemaphoreHandle_t tamaMutex;

#define TFT_eSPI LovyanGFX
#define TFT_eSprite LGFX_Sprite

static constexpr int HOSYOND_LCD_CS = 10;
static constexpr int HOSYOND_LCD_DC = 46;
static constexpr int HOSYOND_LCD_RST = -1;
static constexpr int HOSYOND_LCD_BL = 45;
static constexpr int HOSYOND_SPI_MOSI = 11;
static constexpr int HOSYOND_SPI_MISO = 13;
static constexpr int HOSYOND_SPI_SCLK = 12;
static constexpr int HOSYOND_TOUCH_INT = 17;
static constexpr int HOSYOND_TOUCH_RST = 18;
static constexpr int HOSYOND_BAT_ADC = 9;     // ADC1_CH8, 2:1 divider
static constexpr int HOSYOND_BTN_A = 0;
static constexpr int HOSYOND_AP_ENABLE = 1;
static constexpr int HOSYOND_I2C_SDA = 16;
static constexpr int HOSYOND_I2C_SCL = 15;
static constexpr int HOSYOND_I2S_MCK = 4;
static constexpr int HOSYOND_I2S_BCK = 5;
static constexpr int HOSYOND_I2S_DIN = 6;
static constexpr int HOSYOND_I2S_WS = 7;
static constexpr int HOSYOND_I2S_DOUT = 8;
static constexpr int HOSYOND_TOUCH_PHYSICAL_W = 240;
static constexpr uint8_t HOSYOND_FT6336_ADDR = 0x38;
static constexpr uint8_t HOSYOND_FT6336_TOUCHES_REG = 0x02;
static constexpr uint8_t HOSYOND_FT6336_TOUCH_XH_REG = 0x03;

inline bool hosyondI2CInit() {
  static bool ready = false;
  if (ready) return true;
  const i2c_config_t cfg = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = HOSYOND_I2C_SDA,
    .scl_io_num = HOSYOND_I2C_SCL,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master = {
      .clk_speed = 400000,
    },
  };
  if (i2c_param_config(I2C_NUM_0, &cfg) != ESP_OK) return false;
  esp_err_t err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;
  ready = true;
  return true;
}

struct RTC_TimeTypeDef {
  uint8_t Hours;
  uint8_t Minutes;
  uint8_t Seconds;
};

struct RTC_DateTypeDef {
  uint8_t WeekDay;
  uint8_t Month;
  uint8_t Date;
  uint16_t Year;
};

class HosyondGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;

public:
  HosyondGFX() {
    auto bus_cfg = _bus.config();
    bus_cfg.spi_host = SPI2_HOST;
    bus_cfg.spi_mode = 0;
    bus_cfg.freq_write = 40000000;
    bus_cfg.freq_read = 20000000;
    bus_cfg.spi_3wire = false;
    bus_cfg.use_lock = true;
    bus_cfg.dma_channel = SPI_DMA_CH_AUTO;
    bus_cfg.pin_sclk = HOSYOND_SPI_SCLK;
    bus_cfg.pin_mosi = HOSYOND_SPI_MOSI;
    bus_cfg.pin_miso = HOSYOND_SPI_MISO;
    bus_cfg.pin_dc = HOSYOND_LCD_DC;
    _bus.config(bus_cfg);
    _panel.setBus(&_bus);

    auto panel_cfg = _panel.config();
    panel_cfg.pin_cs = HOSYOND_LCD_CS;
    panel_cfg.pin_rst = HOSYOND_LCD_RST;
    panel_cfg.pin_busy = -1;
    panel_cfg.panel_width = 240;
    panel_cfg.panel_height = 320;
    panel_cfg.offset_x = 0;
    panel_cfg.offset_y = 0;
    panel_cfg.offset_rotation = 2;
    panel_cfg.dummy_read_pixel = 8;
    panel_cfg.dummy_read_bits = 1;
    panel_cfg.readable = true;
    panel_cfg.invert = true;
    panel_cfg.rgb_order = false;
    panel_cfg.dlen_16bit = false;
    panel_cfg.bus_shared = false;
    _panel.config(panel_cfg);

    auto light_cfg = _light.config();
    light_cfg.pin_bl = HOSYOND_LCD_BL;
    light_cfg.invert = false;
    light_cfg.freq = 2000;
    light_cfg.pwm_channel = 7;
    _light.config(light_cfg);
    _panel.setLight(&_light);

    setPanel(&_panel);
  }
};

using M5GFX = HosyondGFX;
inline HosyondGFX* hosyondDisplay = nullptr;

class Button_Compat {
public:
  explicit Button_Compat(int pin = -1, bool active_low = true) : _pin(pin), _activeLow(active_low) {}

  void begin() {
    if (_pin >= 0) pinMode(_pin, _activeLow ? INPUT_PULLUP : INPUT);
    _lastRaw = rawPressed();
    _stable = _lastRaw;
  }

  void update(uint32_t now = millis()) {
    bool raw = rawPressed();
    if (raw != _lastRaw) {
      _lastRaw = raw;
      _lastChange = now;
    }
    if ((now - _lastChange) >= 20 && raw != _stable) {
      bool old = _stable;
      _stable = raw;
      if (_stable) {
        _pressedAt = now;
        _longReported = false;
      } else if (old) {
        _released = true;
        _clicked = (now - _pressedAt) < 600;
      }
    }
  }

  bool isPressed() const { return _stable; }

  bool wasReleased() {
    bool v = _released;
    _released = false;
    return v;
  }

  bool wasClicked() {
    bool v = _clicked;
    _clicked = false;
    return v;
  }

  bool pressedFor(uint32_t ms) {
    if (!_stable || _longReported || millis() - _pressedAt < ms) return false;
    _longReported = true;
    return true;
  }

private:
  bool rawPressed() const {
    if (_pin < 0) return false;
    int v = digitalRead(_pin);
    return _activeLow ? (v == LOW) : (v == HIGH);
  }

  int _pin;
  bool _activeLow;
  bool _lastRaw = false;
  bool _stable = false;
  bool _released = false;
  bool _clicked = false;
  bool _longReported = false;
  uint32_t _lastChange = 0;
  uint32_t _pressedAt = 0;
};

class HosyondTouch_Compat {
public:
  void begin() {
    pinMode(HOSYOND_TOUCH_INT, INPUT_PULLUP);
    hosyondI2CInit();
  }

  void update(uint32_t = millis()) {
    _pressed = false;
    _orientedX = 0;
    if (!hosyondI2CInit()) return;

    uint8_t reg = HOSYOND_FT6336_TOUCHES_REG;
    uint8_t touches = 0;
    esp_err_t err = i2c_master_write_read_device(
      I2C_NUM_0,
      HOSYOND_FT6336_ADDR,
      &reg,
      1,
      &touches,
      1,
      pdMS_TO_TICKS(5)
    );
    if (err != ESP_OK) return;

    touches &= 0x0F;
    if (touches == 0 || touches > 2) return;

    reg = HOSYOND_FT6336_TOUCH_XH_REG;
    uint8_t data[4] = {0};
    err = i2c_master_write_read_device(
      I2C_NUM_0,
      HOSYOND_FT6336_ADDR,
      &reg,
      1,
      data,
      sizeof(data),
      pdMS_TO_TICKS(5)
    );
    if (err != ESP_OK) return;

    uint16_t rawX = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
    if (rawX >= HOSYOND_TOUCH_PHYSICAL_W) rawX = HOSYOND_TOUCH_PHYSICAL_W - 1;

    _orientedX = rawX;
    _pressed = true;
  }

  bool leftPressed() const { return _pressed && _orientedX < HOSYOND_TOUCH_PHYSICAL_W / 2; }
  bool rightPressed() const { return _pressed && _orientedX >= HOSYOND_TOUCH_PHYSICAL_W / 2; }

private:
  bool _pressed = false;
  uint16_t _orientedX = 0;
};

inline HosyondTouch_Compat hosyondTouch;

enum class HosyondTouchZone {
  Left,
  Right,
};

class TouchZoneButton_Compat {
public:
  explicit TouchZoneButton_Compat(HosyondTouchZone zone = HosyondTouchZone::Left) : _zone(zone) {}

  void begin() {
    _lastRaw = rawPressed();
    _stable = _lastRaw;
  }

  void update(uint32_t now = millis()) {
    bool raw = rawPressed();
    if (raw != _lastRaw) {
      _lastRaw = raw;
      _lastChange = now;
    }
    if ((now - _lastChange) >= 20 && raw != _stable) {
      bool old = _stable;
      _stable = raw;
      if (_stable) {
        _pressedAt = now;
        _longReported = false;
      } else if (old) {
        _released = true;
        _clicked = (now - _pressedAt) < 600;
      }
    }
  }

  bool isPressed() const { return _stable; }

  bool wasReleased() {
    bool v = _released;
    _released = false;
    return v;
  }

  bool wasClicked() {
    bool v = _clicked;
    _clicked = false;
    return v;
  }

  bool pressedFor(uint32_t ms) {
    if (!_stable || _longReported || millis() - _pressedAt < ms) return false;
    _longReported = true;
    return true;
  }

private:
  bool rawPressed() const {
    return _zone == HosyondTouchZone::Left ? hosyondTouch.leftPressed() : hosyondTouch.rightPressed();
  }

  HosyondTouchZone _zone;
  bool _lastRaw = false;
  bool _stable = false;
  bool _released = false;
  bool _clicked = false;
  bool _longReported = false;
  uint32_t _lastChange = 0;
  uint32_t _pressedAt = 0;
};

class RTC_Compat {
public:
  void GetTime(RTC_TimeTypeDef* time) {
    time_t now = ::time(nullptr);
    struct tm tmv;
    localtime_r(&now, &tmv);
    time->Hours = tmv.tm_hour;
    time->Minutes = tmv.tm_min;
    time->Seconds = tmv.tm_sec;
  }

  void GetDate(RTC_DateTypeDef* date) {
    time_t now = ::time(nullptr);
    struct tm tmv;
    localtime_r(&now, &tmv);
    date->WeekDay = tmv.tm_wday;
    date->Month = tmv.tm_mon + 1;
    date->Date = tmv.tm_mday;
    date->Year = tmv.tm_year + 1900;
  }

  void SetTime(RTC_TimeTypeDef*) {}
  void SetDate(RTC_DateTypeDef*) {}
};

class IMU_Compat {
public:
  int Init() { return 0; }
  bool getAccelData(float*, float*, float*) { return false; }
};

inline bool hosyondAudioInit() {
  static bool ready = false;
  if (ready) return true;
  if (!hosyondI2CInit()) return false;

  pinMode(HOSYOND_AP_ENABLE, OUTPUT);
  digitalWrite(HOSYOND_AP_ENABLE, LOW);

  i2s_config_t i2s_cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
    .sample_rate = EXAMPLE_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = EXAMPLE_MCLK_FREQ_HZ,
  };

  esp_err_t err = i2s_driver_install(I2S_NUM_1, &i2s_cfg, 0, nullptr);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;

  i2s_pin_config_t pin_cfg = {
    .mck_io_num = HOSYOND_I2S_MCK,
    .bck_io_num = HOSYOND_I2S_BCK,
    .ws_io_num = HOSYOND_I2S_WS,
    .data_out_num = HOSYOND_I2S_DOUT,
    .data_in_num = HOSYOND_I2S_DIN,
  };
  err = i2s_set_pin(I2S_NUM_1, &pin_cfg);
  if (err != ESP_OK) return false;

  err = es8311_codec_init();
  if (err != ESP_OK) return false;

  i2s_zero_dma_buffer(I2S_NUM_1);
  ready = true;
  return true;
}

class Beep_Compat {
public:
  void begin() {
    hosyondAudioInit();
  }
  void update() {}

  void tone(uint16_t freq, uint16_t dur) {
    if (freq == 0 || dur == 0 || !hosyondAudioInit()) return;

    static uint32_t phase = 0;
    constexpr size_t CHUNK_FRAMES = 128;
    int16_t samples[CHUNK_FRAMES * 2];
    uint32_t total = ((uint32_t)EXAMPLE_SAMPLE_RATE * dur) / 1000;
    uint32_t step = ((uint32_t)freq << 16) / EXAMPLE_SAMPLE_RATE;

    while (total > 0) {
      size_t frames = total > CHUNK_FRAMES ? CHUNK_FRAMES : total;
      for (size_t i = 0; i < frames; i++) {
        phase += step;
        int16_t v = (phase & 0x8000) ? 5000 : -5000;
        samples[i * 2] = v;
        samples[i * 2 + 1] = v;
      }
      size_t bytesWritten = 0;
      i2s_write(I2S_NUM_1, samples, frames * 2 * sizeof(int16_t), &bytesWritten, pdMS_TO_TICKS(dur + 20));
      total -= frames;
    }

    for (size_t i = 0; i < 32; i++) {
      samples[i * 2] = 0;
      samples[i * 2 + 1] = 0;
    }
    size_t bytesWritten = 0;
    i2s_write(I2S_NUM_1, samples, 32 * 2 * sizeof(int16_t), &bytesWritten, pdMS_TO_TICKS(20));
  }
};

class AXP192_Compat {
public:
  void ScreenBreath(uint8_t brightness) {
    if (hosyondDisplay) hosyondDisplay->setBrightness((uint8_t)(brightness * 2.55f));
  }

  void SetLDO2(bool state) {
    if (hosyondDisplay) hosyondDisplay->setBrightness(state ? 128 : 0);
  }

  void PowerOff() {
    if (hosyondDisplay) hosyondDisplay->setBrightness(0);
    esp_deep_sleep_start();
  }

  float GetBatVoltage() {
    return readBatteryMilliVolts() / 1000.0f;
  }

  float GetBatCurrent() { return 0.0f; }

  int GetBatLevel() {
    int mv = readBatteryMilliVolts();
    if (mv <= 2500) return 0;
    if (mv >= 4200) return 100;
    return (mv - 2500) / 17;
  }

  float GetVBusVoltage() { return 0.0f; }
  float GetTempInAXP192() { return -999.0f; }
  uint8_t GetBtnPress() { return 0; }

private:
  int readBatteryMilliVolts() const {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) sum += analogReadMilliVolts(HOSYOND_BAT_ADC);
    return (int)((sum / 8) * 2);
  }
};

class Mic_Compat {
public:
  struct config_t {
    uint8_t magnification = 32;
  };

  config_t config() const { return _cfg; }
  void config(const config_t& cfg) { _cfg = cfg; }

  bool begin() {
    if (_begun) return true;
    if (!hosyondAudioInit()) return false;
    _begun = true;
    return true;
  }

  void end() {
    _recording = false;
  }

  bool record(int16_t* buffer, size_t samples, uint32_t rate) {
    if (!buffer || samples == 0) return false;
    if (!_begun && !begin()) return false;
    _dst = buffer;
    _samples = samples;
    _written = 0;
    _rate = rate;
    _recording = true;
    xTaskCreatePinnedToCore(recordTaskThunk, "hosyMicRec", 4096, this, 5, nullptr, 0);
    return true;
  }

  bool isRecording() const { return _recording; }

private:
  static void recordTaskThunk(void* arg) {
    static_cast<Mic_Compat*>(arg)->recordTask();
  }

  void recordTask() {
    int16_t tmp[512];
    while (_recording && _written < _samples) {
      size_t bytesRead = 0;
      esp_err_t err = i2s_read(I2S_NUM_1, tmp, sizeof(tmp), &bytesRead, pdMS_TO_TICKS(1000));
      if (err != ESP_OK || bytesRead == 0) continue;
      size_t words = bytesRead / sizeof(int16_t);
      for (size_t i = 0; i + 1 < words && _written < _samples; i += 2) {
        _dst[_written++] = tmp[i];
      }
    }
    _recording = false;
    vTaskDelete(nullptr);
  }

  config_t _cfg;
  volatile bool _recording = false;
  bool _begun = false;
  int16_t* _dst = nullptr;
  size_t _samples = 0;
  size_t _written = 0;
  uint32_t _rate = EXAMPLE_SAMPLE_RATE;
};

class M5StickCPlus_Compat {
public:
  HosyondGFX Lcd;
  IMU_Compat Imu;
  RTC_Compat Rtc;
  TouchZoneButton_Compat BtnA = TouchZoneButton_Compat(HosyondTouchZone::Left);
  TouchZoneButton_Compat BtnB = TouchZoneButton_Compat(HosyondTouchZone::Right);
  Button_Compat BtnPWR = Button_Compat(-1, true);
  Mic_Compat Mic;
  AXP192_Compat Axp;
  Beep_Compat Beep;

  void begin() {
    Serial.begin(115200);
    delay(10);
    pinMode(HOSYOND_TOUCH_RST, OUTPUT);
    digitalWrite(HOSYOND_TOUCH_RST, LOW);
    delay(10);
    digitalWrite(HOSYOND_TOUCH_RST, HIGH);
    pinMode(HOSYOND_BAT_ADC, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(HOSYOND_BAT_ADC, ADC_11db);
    hosyondTouch.begin();
    BtnA.begin();
    BtnB.begin();
    Lcd.init();
    hosyondDisplay = &Lcd;
    Lcd.setRotation(0);
    Lcd.setBrightness(128);
    Lcd.fillScreen(0);
  }

  void update() {
    uint32_t now = millis();
    hosyondTouch.update(now);
    BtnA.update(now);
    BtnB.update(now);
  }
};

inline M5StickCPlus_Compat M5_compat;
#define M5 M5_compat

#endif
