/*
 * LED Face Mask - Audio Reactive (voice controlled)
 * ---------------------------------------------------
 * Board:   Seeed XIAO ESP32-S3
 * Mic:     INMP441 (I2S digital MEMS microphone)
 * LEDs:    WS2812B, 2.7mm ultra-narrow strip, 160 LEDs/m, ~1.5m -> 240 pixels
 *
 * Behavior:
 *  - Reads the mic continuously, computes a smoothed volume envelope (RMS -> dB).
 *  - Maps that envelope through a compressive curve so that normal speaking
 *    volume stays low, and only very loud volume approaches the intensity
 *    ceiling (MAX_LEVEL, default 0.80 = 80%).
 *  - Drives a slowly color-cycling effect whose BRIGHTNESS follows the voice
 *    envelope. Swap out `renderEffect()` for a different visual if desired.
 *  - A hardware-safety power budget (FastLED.setMaxPowerInVoltsAndMilliamps)
 *    is layered on top as a second, independent safety net.
 *
 * Library requirements (Arduino IDE Library Manager):
 *  - FastLED
 *  - Board package: esp32 by Espressif Systems.
 *    This sketch uses the legacy `driver/i2s.h` API, which is present and
 *    works on both the 2.x and 3.x Arduino-ESP32 core lines (on 3.x it is
 *    kept for backward compatibility). If your installed core ever drops it,
 *    switch to the newer `ESP_I2S.h` / I2SClass API instead - the RMS/curve
 *    logic below does not need to change.
 *
 * Wiring (XIAO ESP32-S3 pin labels):
 *   LED strip DIN  -> D0   (GPIO1)   (+ 330-470ohm resistor in series, right
 *                                      at the strip's input pigtail)
 *   LED strip V+   -> Battery+ (direct, no boost converter - see README)
 *   LED strip GND  -> Battery- / GND (shared with board GND)
 *   Small cap (100-220uF) across V+/GND right at the strip's first pixel.
 *
 *   INMP441 VDD    -> 3V3
 *   INMP441 GND    -> GND
 *   INMP441 L/R    -> GND      (selects left channel)
 *   INMP441 WS     -> D1  (GPIO2)   (word select / LRCLK)
 *   INMP441 SD     -> D2  (GPIO3)   (data out from mic)
 *   INMP441 SCK    -> D3  (GPIO4)   (bit clock)
 *
 * Calibration:
 *   Open Serial Monitor at 115200 baud. It prints the live dB reading.
 *   1. Stay silent, note the "quiet" dB value -> set NOISE_FLOOR_DB a couple
 *      dB above it (so background hiss doesn't light up the mask).
 *   2. Talk normally, note the dB value -> this should map to a low/medium
 *      level with the default curve.
 *   3. Talk as loud/shout as you expect in real use -> set LOUD_DB to that
 *      value. That is the point that reaches MAX_LEVEL (80% by default).
 */

#include <FastLED.h>
#include "driver/i2s.h"

// ---------------------------------------------------------------------------
// LED CONFIG
// ---------------------------------------------------------------------------
#define LED_PIN         1     // D0
#define NUM_LEDS        240   // 1.5m * 160 LEDs/m
#define COLOR_ORDER     GRB   // swap to RGB if colors look wrong on your strip
#define LED_CHIPSET     WS2812B

// Hard safety net independent of the software-level 80% cap below.
// Tune to your battery's real max continuous discharge current (datasheet
// C-rating x capacity), leaving headroom - e.g. a 400mAh/1C cell -> ~400mA
// available for everything; a bigger cell you might swap to -> raise this.
#define LED_SUPPLY_VOLTS      5
#define MAX_LED_MILLIAMPS     700

// ---------------------------------------------------------------------------
// MIC / I2S CONFIG
// ---------------------------------------------------------------------------
#define I2S_WS_PIN      2     // D1 - word select (LRCLK)
#define I2S_SD_PIN      3     // D2 - data in from mic
#define I2S_SCK_PIN     4     // D3 - bit clock

#define I2S_PORT        I2S_NUM_0
#define SAMPLE_RATE     16000
#define SAMPLE_BITS     32                 // INMP441 outputs 24-bit data left-justified in a 32-bit frame
#define READ_BLOCK_SAMPLES  256            // samples per RMS window (~16ms @ 16kHz)

// ---------------------------------------------------------------------------
// VOICE -> BRIGHTNESS MAPPING
// ---------------------------------------------------------------------------
// dB values below are on an arbitrary internal scale (20*log10(rms)), not
// calibrated SPL - calibrate the two constants below using Serial Monitor.
float NOISE_FLOOR_DB = 40.0f;   // "silence" - below this, level = 0
float LOUD_DB        = 85.0f;   // "shouting" - at/above this, level = 1.0 (pre-cap)

#define CURVE_EXPONENT   2.5f   // >1 compresses low/medium volume down;
                                 // only the top of the range climbs fast.
#define MAX_LEVEL        0.80f  // hard ceiling: even max volume caps here.

// Envelope follower smoothing (fast attack, slower decay -> natural pulse).
#define ATTACK_ALPHA     0.60f
#define DECAY_ALPHA      0.12f

// ---------------------------------------------------------------------------

CRGB leds[NUM_LEDS];
int32_t i2sReadBuffer[READ_BLOCK_SAMPLES];

float smoothedLevel = 0.0f;   // 0..MAX_LEVEL, drives the effect
uint8_t hue = 0;

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = (i2s_bits_per_sample_t)SAMPLE_BITS,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = READ_BLOCK_SAMPLES,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

// Reads one block of samples and returns their RMS amplitude (0..~2^31).
float readMicRMS() {
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, (void*)i2sReadBuffer, sizeof(i2sReadBuffer), &bytesRead, portMAX_DELAY);

  int samplesRead = bytesRead / sizeof(int32_t);
  if (samplesRead <= 0) return 0.0f;

  double sumSquares = 0.0;
  for (int i = 0; i < samplesRead; i++) {
    // INMP441 24-bit sample is left-justified in the 32-bit word; shift down.
    int32_t sample = i2sReadBuffer[i] >> 8;
    sumSquares += (double)sample * (double)sample;
  }
  double meanSquare = sumSquares / samplesRead;
  return (float)sqrt(meanSquare);
}

// Converts RMS amplitude to a 0..1 level using NOISE_FLOOR_DB..LOUD_DB,
// then applies the compressive curve.
float rmsToLevel(float rms) {
  float db = (rms > 1.0f) ? 20.0f * log10f(rms) : 0.0f;

  // Uncomment while calibrating:
  // Serial.println(db);

  float normalized = (db - NOISE_FLOOR_DB) / (LOUD_DB - NOISE_FLOOR_DB);
  normalized = constrain(normalized, 0.0f, 1.0f);

  float curved = powf(normalized, CURVE_EXPONENT);
  return curved * MAX_LEVEL;
}

// Smooth the raw level with an asymmetric attack/decay envelope follower.
float smoothLevel(float target, float current) {
  float alpha = (target > current) ? ATTACK_ALPHA : DECAY_ALPHA;
  return current + alpha * (target - current);
}

// Default visual: slow color cycle, brightness follows the voice envelope.
// Replace this with any other effect - `smoothedLevel` (0..MAX_LEVEL) is
// the only thing you need to read.
void renderEffect() {
  hue++;  // slow hue rotation over time regardless of volume

  uint8_t brightness = (uint8_t)(smoothedLevel * 255.0f);
  CRGB color = CHSV(hue, 255, 255);

  fill_solid(leds, NUM_LEDS, color);
  FastLED.setBrightness(brightness);
  FastLED.show();
}

void setup() {
  Serial.begin(115200);

  setupI2S();

  FastLED.addLeds<LED_CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(LED_SUPPLY_VOLTS, MAX_LED_MILLIAMPS);
  FastLED.setBrightness(0);
  FastLED.clear();
  FastLED.show();
}

void loop() {
  float rms = readMicRMS();
  float targetLevel = rmsToLevel(rms);
  smoothedLevel = smoothLevel(targetLevel, smoothedLevel);

  renderEffect();
}
