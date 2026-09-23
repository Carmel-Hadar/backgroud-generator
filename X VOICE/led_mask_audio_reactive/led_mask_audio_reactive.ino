/*
 * X Voice - Audio Reactive LED Face Mask (voice controlled)
 * ---------------------------------------------------------
 * Board:   Seeed XIAO nRF52840 Sense
 * Mic:     onboard PDM MEMS microphone (no external wiring - built into
 *          the Sense board itself)
 * LEDs:    WS2812B, 2.7mm ultra-narrow strip, 160 LEDs/m, ~1.5m -> 240 pixels
 *
 * Behavior:
 *  - Reads the onboard mic continuously (PDM, mono, 16kHz), computes a
 *    smoothed volume envelope (RMS -> dB).
 *  - Maps that envelope through a compressive curve so that normal speaking
 *    volume stays low, and only very loud volume approaches the intensity
 *    ceiling (MAX_LEVEL, default 0.80 = 80%).
 *  - Drives a slowly color-cycling effect whose BRIGHTNESS follows the voice
 *    envelope. Swap out `renderEffect()` for a different visual if desired.
 *
 * Library requirements (Arduino IDE Library Manager):
 *  - Adafruit NeoPixel
 *    (FastLED's nRF52840 support is not solid yet - forum reports of
 *    intermittent flicker on the "clockless_arm_nrf" driver on XIAO boards
 *    specifically. Adafruit_NeoPixel is the library the community actually
 *    uses successfully on this board, so that's what this sketch uses.)
 *  - Board package: "Seeed nRF52 mbed-enabled Boards" in Boards Manager
 *    (NOT the older non-mbed "Seeed nRF52 Boards" - the mbed-enabled one is
 *    what ships PDM.h support for the onboard mic). Select
 *    "Seeed XIAO nRF52840 Sense" as the board.
 *  - PDM.h ships with that board package, nothing extra to install for the mic.
 *
 * Wiring (XIAO nRF52840 Sense pin labels):
 *   LED strip DIN  -> D0        (+ 330-470ohm resistor in series, right at
 *                                 the strip's input pigtail)
 *   LED strip V+   -> Battery+  (direct, no boost converter - see README)
 *   LED strip GND  -> Battery- / GND (shared with board GND)
 *   Small cap (100-220uF) across V+/GND right at the strip's first pixel.
 *
 *   Microphone: nothing to wire - it's built into the Sense board.
 *
 * Calibration:
 *   Open Serial Monitor at 115200 baud. It prints the live dB reading.
 *   1. Stay silent, note the "quiet" dB value -> set NOISE_FLOOR_DB a couple
 *      dB above it (so background hiss doesn't light up the mask).
 *   2. Talk normally, note the dB value -> this should map to a low/medium
 *      level with the default curve.
 *   3. Talk as loud/shout as you expect in real use -> set LOUD_DB to that
 *      value. That is the point that reaches MAX_LEVEL (80% by default).
 *   MIC_GAIN below is a second knob (software gain on the PDM input itself,
 *   0-80) - raise it if even shouting barely moves the dB reading, lower it
 *   if the mic clips/flattens out on normal speech.
 */

#include <PDM.h>
#include <Adafruit_NeoPixel.h>

// ---------------------------------------------------------------------------
// LED CONFIG
// ---------------------------------------------------------------------------
#define LED_PIN         D0
#define NUM_LEDS        240   // 1.5m * 160 LEDs/m

// Adafruit_NeoPixel has no automatic per-frame power limiter like FastLED.
// Because renderEffect() below always sets every pixel to the SAME color and
// brightness, total current is a direct, predictable function of MAX_LEVEL -
// so that one constant (further down) is the power budget for this sketch.
// If you change the effect to independently-colored pixels, add your own
// current calculation here before raising MAX_LEVEL.
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------------------------------------------------------------
// MIC / PDM CONFIG
// ---------------------------------------------------------------------------
#define SAMPLE_RATE     16000
#define MIC_GAIN        30     // 0-80, software gain on the PDM input

short pdmBuffer[512];
volatile int pdmSamplesRead = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(pdmBuffer, bytesAvailable);
  pdmSamplesRead = bytesAvailable / 2;   // 16-bit samples
}

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

float smoothedLevel = 0.0f;   // 0..MAX_LEVEL, drives the effect
uint16_t hue16 = 0;

// Computes RMS over whatever PDM samples have arrived since the last call.
// Returns -1 if no new samples are ready yet (caller should skip this tick).
float readMicRMS() {
  if (pdmSamplesRead <= 0) return -1.0f;

  noInterrupts();
  int count = pdmSamplesRead;
  pdmSamplesRead = 0;
  interrupts();

  double sumSquares = 0.0;
  for (int i = 0; i < count; i++) {
    double sample = pdmBuffer[i];
    sumSquares += sample * sample;
  }
  double meanSquare = sumSquares / count;
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
  hue16 += 120;  // slow hue rotation over time regardless of volume

  uint8_t brightness = (uint8_t)(smoothedLevel * 255.0f);
  uint32_t color = strip.gamma32(strip.ColorHSV(hue16, 255, 255));

  strip.fill(color);
  strip.setBrightness(brightness);
  strip.show();
}

void setup() {
  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(0);
  strip.show();

  PDM.onReceive(onPDMdata);
  PDM.setGain(MIC_GAIN);
  if (!PDM.begin(1, SAMPLE_RATE)) {   // mono, 16kHz
    Serial.println("PDM init failed - check board package (needs the mbed-enabled Seeed nRF52 core)");
    while (1) { delay(1000); }
  }
}

void loop() {
  float rms = readMicRMS();
  if (rms < 0.0f) return;   // no new samples yet this tick

  float targetLevel = rmsToLevel(rms);
  smoothedLevel = smoothLevel(targetLevel, smoothedLevel);

  renderEffect();
}
