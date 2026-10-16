/*
 * X Voice - Audio Reactive LED Face Mask (voice controlled)
 * ---------------------------------------------------------
 * Board:   Seeed XIAO nRF52840 Sense
 * Mic:     onboard PDM MEMS microphone (no external wiring - built into
 *          the Sense board itself)
 * LEDs:    WS2812B, 2.7mm ultra-narrow strip, 160 LEDs/m, ~1.5m -> 240 pixels
 *
 * Behavior:
 *  - Reads the onboard mic continuously (PDM, mono, 16kHz), band-pass filters
 *    it to the speech band (VOICE_LOW_HZ..VOICE_HIGH_HZ), and computes a
 *    smoothed volume envelope (RMS -> dB) from that band only.
 *  - Maps that envelope through a compressive curve so that normal speaking
 *    volume stays low, and only very loud volume approaches the intensity
 *    ceiling (MAX_LEVEL, default 0.50 = 50% perceived).
 *  - Effect: a single fixed color (LED_R/LED_G/LED_B) glowing from the X's
 *    center (strip1Center/strip2Center) with a Gaussian falloff along the strips -
 *    louder = brighter center AND a wider spread down the arms.
 *    Swap out `renderEffect()` for a different visual.
 *  - A second, independent ceiling (MAX_LED_MILLIAMPS) caps brightness further
 *    if needed to keep estimated LED current under a hard mA budget - see
 *    powerLimitedBrightness(). MAX_LEVEL and MAX_LED_MILLIAMPS both apply;
 *    whichever is more restrictive at any given moment wins.
 *  - Sleep mode: five minutes without head motion (onboard LSM6DS3 gyro) and
 *    without voice above the noise floor ->
 *    mic off, strip dark, IMU in low-power; an acceleration spike wakes it.
 *  - DEBUG_PLOT_AUDIO (on by default): while USB-connected, prints the raw
 *    mic level and the mapped brightness level every mic read, in a format
 *    the Arduino IDE's Serial Plotter (Tools -> Serial Plotter, 115200 baud)
 *    graphs live - useful for watching the mic react and for calibration.
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
 *  - Seeed Arduino LSM6DS3 (onboard IMU, for sleep mode)
 *
 * Wiring (XIAO nRF52840 Sense pin labels):
 *   LED strip DIN  -> D4        (+ 330-470ohm resistor in series, right at
 *                                 the strip's input pigtail)
 *   LED strip V+   -> Battery+  (direct, no boost converter - see README)
 *   LED strip GND  -> Battery- / GND (shared with board GND)
 *   Small cap (100-220uF) across V+/GND right at the strip's first pixel.
 *
 *   Microphone: nothing to wire - it's built into the Sense board.
 *
 * Calibration:
 *   Open the Serial Plotter (Tools -> Serial Plotter) at 115200 baud - with
 *   DEBUG_PLOT_AUDIO on (default) it graphs two lines live: the raw mic dB
 *   reading, and the mapped brightness level (x100, so both sit in a
 *   comparable range on the same graph).
 *   1. Stay silent, read the "quiet" dB line -> set NOISE_FLOOR_DB a couple
 *      dB above it (so background hiss doesn't light up the mask).
 *   2. Talk normally -> this should map to a low/medium level with the
 *      default curve.
 *   3. Talk as loud/shout as you expect in real use -> set LOUD_DB to that
 *      dB value. That is the point that reaches MAX_LEVEL (50% by default).
 *   MIC_GAIN below is a second knob (software gain on the PDM input itself,
 *   0-80) - raise it if even shouting barely moves the dB reading, lower it
 *   if the mic clips/flattens out on normal speech.
 */

#include <PDM.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <algorithm>   // std::nth_element for the click-gap median

// Mode-switch trigger. 0 = double TAP on the device (accelerometer) - the
// default since a real party: the double tongue-click (1) fired far too often
// on music/crowd noise there. The click code is kept, just not run.
#define USE_CLICK_TRIGGER  0

// ---------------------------------------------------------------------------
// LED CONFIG
// ---------------------------------------------------------------------------
// Strip 1 / strip 2 = the strips as mapped with led_finder.py (59 LEDs
// center 29 / 61 LEDs center 31). Strip 1 is the one wired to D10 + D7.
#define LED_PIN         D10

// Strip power switch: an AO3400A (N-channel, logic level) on the strip's
// GROUND side - gate on STRIP_PWR_PIN. No gate pull-down fitted: the gate
// only floats for a moment at boot / while flashing (strip may glitch then);
// in sleep the pin actively holds it LOW. Each WS2812 draws ~0.3-1mA even when dark, x240 =
// 70-240mA, so without this a "sleeping" mask still drained a 400mAh cell in
// ~1.5-5 hours. In sleep the strip is cut off completely.
#define STRIP_PWR_PIN   D7

// Second strip, same wiring (own AO3400A on its ground).
#define LED2_PIN        D4
#define STRIP2_PWR_PIN  D5      // D5 is free: the IMU is on the internal Wire1 bus, not D4/D5

// The X is two strips, one per diagonal, crossing at the center. Every
// effect works on one virtual strip: strip 1's pixels, then strip 2's
// (virtual pixel p < NUM_LEDS1 -> strip 1 LED p, else strip 2 LED p - NUM_LEDS1).
// Mapped with led_finder.py: strip 1 = LEDs 0-58, center 29; strip 2 = 0-60, center 31.
#define NUM_LEDS1       59
#define NUM_LEDS2       61
#define NUM_LEDS        (NUM_LEDS1 + NUM_LEDS2)   // virtual pixels across both strips

// 1 = hue follows the pitch of the voice (see VOICE PITCH -> HUE below).
// 0 = fixed LED_R/LED_G/LED_B color, only brightness changes with volume.
#define COLOR_BY_PITCH  1

// Fixed color, used when COLOR_BY_PITCH is 0. 255,255,255 = white.
#define LED_R  255
#define LED_G  255
#define LED_B  255

Adafruit_NeoPixel strip(NUM_LEDS1, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(NUM_LEDS2, LED2_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------------------------------------------------------------
// EXPLICIT POWER LIMITER (Adafruit_NeoPixel has no automatic one like FastLED)
// ---------------------------------------------------------------------------
// Hard ceiling on LED current, independent of MAX_LEVEL below. Tune this to
// your actual battery's continuous discharge rating (capacity x C-rating) -
// e.g. a 400mAh cell with a 1-2C rating gives ~400-800mA to work with; a
// bigger battery raises this number accordingly.
#define MAX_LED_MILLIAMPS   700.0f

// mA per "channel count" (0-255) per pixel at 5V, derived from this strip's
// own datasheet rating rather than guessed: 5.76W/m / 160 px/m = 36mW/pixel
// at full white (R=G=B=255, sum of counts = 765) -> 7.2mA/pixel at 5V ->
// 7.2mA / 765 = ~9.4uA per unit of channel count.
#define MICROAMPS_PER_CHANNEL_COUNT   9.4f

// Returns a 0..1 factor to multiply every pixel by so the estimated current
// stays under MAX_LED_MILLIAMPS. `weightSum` is the sum of all per-pixel
// intensity weights (0..1 each) that are about to be applied to a full-scale
// color whose R+G+B is `channelSum`.
float powerLimitFactor(float weightSum, int channelSum) {
  float milliamps = weightSum * channelSum * MICROAMPS_PER_CHANNEL_COUNT / 1000.0f;
  if (milliamps <= MAX_LED_MILLIAMPS) return 1.0f;
  return MAX_LED_MILLIAMPS / milliamps;
}

// ---------------------------------------------------------------------------
// MIC / PDM CONFIG
// ---------------------------------------------------------------------------
#define SAMPLE_RATE     16000
#define MIC_GAIN        30     // 0-80, software gain on the PDM input

short pdmBuffer[512];
volatile int pdmSamplesRead = 0;

void pushPitchSample(float raw);   // defined in VOICE PITCH -> HUE below
void pushClickSample(float raw);   // defined in DOUBLE TONGUE-CLICK TRIGGER below

// ---------------------------------------------------------------------------
// RAW AUDIO CAPTURE (development tool, driven by capture_trigger.py)
// ---------------------------------------------------------------------------
// Send 'R' over serial -> the next CAPTURE_SAMPLES raw mic samples are kept in
// RAM, then dumped as one binary block:
//   "CAPTURE_BEGIN <n>\n" + n int16 little-endian + "\nCAPTURE_END\n"
// Recorded from the board's own mic so the analysis sees exactly what a
// trigger detector running here would see.
#define CAPTURE_SAMPLES  40000   // 2.5s at 16kHz = 80KB of RAM
int16_t captureBuf[CAPTURE_SAMPLES];
volatile int  captureCount = 0;
volatile bool capturing    = false;

// ---------------------------------------------------------------------------
// LIVE STREAM (development tool, driven by stream_session.py)
// ---------------------------------------------------------------------------
// 'S' starts, 'X' stops (auto-stop after STREAM_MAX_MS). While streaming, the
// serial port carries ONLY binary packets - no text lines:
//   0xA5 0x5A <type> <len lo> <len hi> <payload>
//   'A' audio:   uint32 first sample index, int16[] raw mic samples
//   'C' click:   uint32 sample index, float score  (every onset, pass or not)
//   'T' trigger: uint32 sample index
//   'I' imu:     uint32 sample index, float ax, ay, az (g), gyro max (dps)
//   'F' accel:   uint32 sample index, uint8 FIFO overrun, uint8 n, int16 xyz[n]
//                (every 833Hz accel sample, raw: * ACCEL_RAW_TO_G for g)
// Sample indices count from the last startMic(), so audio, click candidates,
// triggers and IMU readings all line up on one timeline. The audio ring
// reuses captureBuf (capture and stream are never on at the same time).
#define STREAM_MAX_MS  240000UL
volatile bool     streaming      = false;
volatile uint32_t micSampleCount = 0;   // all samples since startMic()
uint32_t          streamTail     = 0;
uint32_t          streamStartMs  = 0;

void streamPacket(uint8_t type, const void* a, uint16_t na, const void* b, uint16_t nb) {
  uint16_t len = na + nb;
  uint8_t hdr[5] = {0xA5, 0x5A, type, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  Serial.write(hdr, 5);
  Serial.write((const uint8_t*)a, na);
  if (nb) Serial.write((const uint8_t*)b, nb);
}

void streamDrain() {
  if (!streaming) return;
  uint32_t head = micSampleCount;
  if (head - streamTail > CAPTURE_SAMPLES) streamTail = head - CAPTURE_SAMPLES + 1024;   // fell behind: skip ahead (shows as a gap)
  while (head != streamTail) {
    uint32_t pos = streamTail % CAPTURE_SAMPLES;
    uint32_t n = head - streamTail;
    if (n > 512) n = 512;
    if (n > CAPTURE_SAMPLES - pos) n = CAPTURE_SAMPLES - pos;
    streamPacket('A', &streamTail, 4, &captureBuf[pos], n * 2);
    streamTail += n;
  }
}

void streamEvent(uint8_t type, uint32_t sampleIdx, const float* vals, int nvals) {
  if (!streaming) return;
  streamPacket(type, &sampleIdx, 4, vals, nvals * 4);
}

// The pitch buffer is fed HERE, not from loop(): loop() runs ~35x/s while the
// mic delivers ~60 blocks/s, so loop() only ever sees the latest block. That's
// fine for a volume reading, but pitch detection needs a gap-free signal -
// splicing non-adjacent blocks together destroys the periodicity it looks for.
void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(pdmBuffer, bytesAvailable);
  int n = bytesAvailable / 2;   // 16-bit samples
  for (int i = 0; i < n; i++) {
    pushPitchSample((float)pdmBuffer[i]);
#if USE_CLICK_TRIGGER
    pushClickSample((float)pdmBuffer[i]);
#endif
    if (streaming) captureBuf[micSampleCount % CAPTURE_SAMPLES] = pdmBuffer[i];
    micSampleCount++;
  }
  if (capturing) {
    for (int i = 0; i < n && captureCount < CAPTURE_SAMPLES; i++) captureBuf[captureCount++] = pdmBuffer[i];
    if (captureCount >= CAPTURE_SAMPLES) capturing = false;
  }
  pdmSamplesRead = n;
}

// ---------------------------------------------------------------------------
// VOICE -> BRIGHTNESS MAPPING
// ---------------------------------------------------------------------------
// dB values below are on an arbitrary internal scale (20*log10(rms)), not
// calibrated SPL - calibrate the two constants below using Serial Monitor.
// Calibrated 2026-09-27 with the voice-band filter on (quiet / talk / shout):
//   quiet: p50 17, p99 24     talk: p50 33, p90 45     shout: p50 52, p99 55
// Quiet p99 is ~24dB; at 26 breathing/rustling (28-38dB) kept the mask lit
// ~60% of the time, 34 needed speech that was too loud. 30 is in between.
float NOISE_FLOOR_DB = 30.0f;   // "silence" - below this, level = 0
// Guided calibration 2026-09-27: quiet p99 25, talk p90 44, shout p50 51 /
// p99 56. At 64 even a shout never got the spread past ~1/3 of the arms.
// Recalibrated 2026-10-13: quiet p50 19, talk p50 35 / p90 48, high p90 52,
// shout p50 54 / p99 60. At 56 a high voice already hit full spread/speed.
float LOUD_DB        = 60.0f;   // "shouting" - at/above this, level = 1.0 (pre-cap)

// "Sound floor" profile, toggled by 5 taps: for loud music, where the
// voice thresholds above sit permanently at the top. Both ends move up so the
// music's dynamics map onto the full range instead of saturating.
// Scale estimate (not yet measured with music): mic -26dBFS @ 94dB SPL, PDM
// gain 30 = -5dB -> internal dB ~= SPL - 35, before the voice-band filter takes
// out the bass. Full-scale clipping sits at ~87-90 here (~120dB SPL), so a
// 100-110dB SPL dance floor should read ~60-75. Check with serial_plotter.py.
// The voice pair = the NOISE_FLOOR_DB / LOUD_DB defaults above, keep in sync.
#define VOICE_NOISE_FLOOR_DB   30.0f
#define VOICE_LOUD_DB          60.0f
#define FLOOR_NOISE_FLOOR_DB   50.0f
#define FLOOR_LOUD_DB          75.0f
bool soundFloor = false;

void setSoundFloor(bool on) {
  soundFloor = on;
  NOISE_FLOOR_DB = on ? FLOOR_NOISE_FLOOR_DB : VOICE_NOISE_FLOOR_DB;
  LOUD_DB        = on ? FLOOR_LOUD_DB        : VOICE_LOUD_DB;
}

// Voice level that keeps the mask awake. Higher than NOISE_FLOOR_DB on
// purpose: quiet rooms still throw occasional 30-39dB spikes, and at the
// noise floor those would reset the idle clock forever and never let it sleep.
#define ACTIVITY_DB      35.0f

#define CURVE_EXPONENT   0.8f   // >1 compresses low/medium volume down;
                                 // <1 = log-like: quiet speech already lights
                                 // up, loud volume compresses.
                                 // (LED_GAMMA already handles the eye's curve.)
#define MAX_LEVEL        0.50f  // hard ceiling: even max volume caps here
                                 // (perceived level - see LED_GAMMA below).

// Envelope follower smoothing (fast attack, slower decay -> natural pulse).
#define ATTACK_ALPHA     0.30f   // was 0.60 - chased every syllable peak
#define DECAY_ALPHA      0.30f   // was 0.12 (~450ms fade) -> ~160ms, snappier

// ---------------------------------------------------------------------------
// SERIAL PLOTTER (for tuning while the board is plugged into a computer)
// ---------------------------------------------------------------------------
// 1 = print [dB, mapped level x100, pitch Hz (0 = unvoiced)] every mic read, for Arduino IDE's
// Serial Plotter (Tools -> Serial Plotter, 115200 baud) to graph live.
// Safe to leave on: guarded by `if (Serial)`, so it costs nothing when the
// mask is running standalone with no computer attached. Set to 0 to disable
// entirely once you're done tuning.
#define DEBUG_PLOT_AUDIO   1

// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SLEEP MODE (motion-based, ported from The Wizard's XIAO_IMU)
// ---------------------------------------------------------------------------
// When the mask isn't moving, there's no point listening and driving LEDs.
// Awake: IDLE_SLEEP_MS with no rotation above WAKE_GYRO_DPS and no voice
// above ACTIVITY_DB -> sleep (talking with a still head keeps it awake). Asleep: gyro off, mic off, strip dark,
// loop polls at 25Hz; a DOUBLE TAP wakes it (the same detector as the mode
// switch, see DOUBLE TAP TRIGGER). The accel stays at 833Hz with its FIFO on,
// since a few-ms tap falls between 26Hz samples; at ~0.15mA that costs ~1%
// of the battery a day. (It used to wake on any |a|-1g spike above 0.5g,
// which bumps, picking it up or moving its bag all did.)
#define WAKE_GYRO_DPS     50.0f
#define IDLE_SLEEP_MS     300000UL  // five minutes without motion or voice -> sleep
#define GYRO_CHECK_MS     10        // awake: motion check at 100Hz
#define SLEEP_SAMPLE_MS   40        // asleep: 25Hz (~36 FIFO samples per poll)

// After an ODR / power-mode change the LSM6DS3 outputs garbage for a few
// cycles (saturated 2000dps readings were measured). Without this window the
// first sample after entering sleep crosses every threshold and wakes it.
#define IMU_SETTLE_MS     100

LSM6DS3 imu(I2C_MODE, 0x6A);
bool     imuOk          = false;   // IMU failed to init -> never sleep
bool     asleep         = false;
uint32_t lastActivityMs = 0;       // last head motion OR voice above the noise floor
uint32_t lastGyroCheck  = 0;
uint32_t imuSettleUntil = 0;

// Read-modify-write, not a full write: the same registers hold the full-scale
// ranges, and overwriting them would silently change every reading's scale.
#define REG_CTRL1_XL  0x10   // [7:4] accel ODR
#define REG_CTRL2_G   0x11   // [7:4] gyro ODR
#define REG_CTRL6_C   0x15   // bit4 XL_HM_MODE: 1 = high-performance off
#define REG_CTRL7_G   0x16   // bit7 G_HM_MODE:  1 = high-performance off
#define ODR_OFF       0x0
#define ODR_26HZ      0x2
#define ODR_416HZ     0x6
#define ODR_833HZ     0x7

void imuConfigure(uint8_t xlOdr, uint8_t gOdr, bool lowPower) {
  uint8_t v;
  imu.readRegister(&v, REG_CTRL1_XL);
  imu.writeRegister(REG_CTRL1_XL, (uint8_t)((v & 0x0F) | (xlOdr << 4)));
  imu.readRegister(&v, REG_CTRL2_G);
  imu.writeRegister(REG_CTRL2_G,  (uint8_t)((v & 0x0F) | (gOdr << 4)));
  imu.readRegister(&v, REG_CTRL6_C);
  imu.writeRegister(REG_CTRL6_C, (uint8_t)(lowPower ? (v | 0x10) : (v & ~0x10)));
  imu.readRegister(&v, REG_CTRL7_G);
  imu.writeRegister(REG_CTRL7_G, (uint8_t)(lowPower ? (v | 0x80) : (v & ~0x80)));
  imuSettleUntil = millis() + IMU_SETTLE_MS;
}

// ---------------------------------------------------------------------------
// ACCEL FIFO (for tap detection)
// ---------------------------------------------------------------------------
// A tap on the mask is a jolt lasting a few ms. loop() only gets to read the
// IMU ~30x/s, which would miss almost all of them, so the IMU's own FIFO
// collects every accelerometer sample at 833Hz and loop() reads the backlog.
// The FIFO holds ~0.8s of xyz at this rate, plenty for a ~30ms loop.
// Accel only (no gyro) in the FIFO, so the word pattern is X, Y, Z, X, ...
#define REG_FIFO_CTRL3    0x08   // [5:3] gyro decimation, [2:0] accel decimation
#define REG_FIFO_CTRL5    0x0A   // [6:3] FIFO ODR, [2:0] mode
#define REG_FIFO_STATUS1  0x3A   // unread words, low byte
#define REG_FIFO_OUT_L    0x3E
#define FIFO_ODR_833      0x7
#define FIFO_MODE_STREAM  0x6    // continuous: oldest data is overwritten when full
#define ACCEL_RAW_TO_G    (0.061f * 8.0f / 1000.0f)   // +-16g full scale: 0.488 mg/LSB

void fifoStart() {
  imu.writeRegister(REG_FIFO_CTRL5, 0x00);                  // bypass = clear it
  imu.writeRegister(REG_FIFO_CTRL3, 0x01);                  // accel, no decimation; no gyro
  imu.writeRegister(REG_FIFO_CTRL5, (FIFO_ODR_833 << 3) | FIFO_MODE_STREAM);
}

void fifoStop() {
  imu.writeRegister(REG_FIFO_CTRL5, 0x00);
}

bool fifoBurst = true;    // read xyz in one 6-byte transaction (the address wraps
                          // within FIFO_DATA_OUT); 'B' toggles per-word reads
bool fifoMargin = true;   // test toggle ('M'): leave the newest sample in the FIFO
uint32_t fifoErrors = 0;  // short / failed I2C reads (debug 'D' prints it)

// I2C read that reports a short read. The library's readRegisterRegion()
// silently accepts fewer bytes than asked for - with the FIFO that means a
// word was popped but never stored, and every axis after it shifts by one
// (gravity hopping z -> y -> x, ~0.6 fake 1.3g jolts per second).
bool imuReadChecked(uint8_t reg, uint8_t* buf, int len) {
  Wire1.beginTransmission(0x6A);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  int got = Wire1.requestFrom(0x6A, len);
  if (got != len) {
    while (Wire1.available()) Wire1.read();
    return false;
  }
  for (int i = 0; i < len; i++) buf[i] = Wire1.read();
  return true;
}

// Reads up to maxSamples xyz samples (raw int16) from the FIFO. Returns the
// count; *overrun is set if the FIFO had filled up (samples were lost).
int fifoRead(int16_t* out, int maxSamples, bool* overrun) {
  uint8_t st[4];
  *overrun = false;
  if (!imuReadChecked(REG_FIFO_STATUS1, st, 4)) { fifoErrors++; return 0; }
  int words   = st[0] | ((st[1] & 0x07) << 8);
  int pattern = st[2] | ((st[3] & 0x03) << 8);           // which axis the next word is
  *overrun = (st[1] & 0x40) != 0;
  // A completely full FIFO (2048 words) reads back as a count of 0 with the
  // FULL_SMART flag set - without this it looked empty and was never drained.
  if (words == 0 && (st[1] & 0x20)) words = 2048;
  int16_t junk;
  while (pattern % 3 != 0 && words > 0) {                 // realign to an X word
    if (!imuReadChecked(REG_FIFO_OUT_L, (uint8_t*)&junk, 2)) { fifoErrors++; return 0; }
    pattern++; words--;
  }
  // Never take the newest sample: it may still be mid-write (x landed, y/z
  // not yet), and reading it shifted the axes by one word - gravity hopping
  // from z to y showed up as ~0.6 fake 1.3g "jolts" per second.
  int n = words / 3 - (fifoMargin ? 1 : 0);
  if (n < 0) n = 0;
  if (n > maxSamples) n = maxSamples;
  for (int k = 0; k < n; k++) {
    bool ok = true;
    if (fifoBurst) {
      ok = imuReadChecked(REG_FIFO_OUT_L, (uint8_t*)&out[3 * k], 6);   // one transaction per sample
    } else {
      for (int ax = 0; ax < 3 && ok; ax++) ok = imuReadChecked(REG_FIFO_OUT_L, (uint8_t*)&out[3 * k + ax], 2);
    }
    // A failed read may have popped words we never got: keep only the samples
    // before it; the next call realigns on the pattern counter, which tracks
    // what the FIFO actually popped.
    if (!ok) { fifoErrors++; return k; }
  }
  return n;
}

// Short, non-blocking blink of the onboard RGB LED on each transition
// (blue = sleep, green = wake). Onboard LEDs are active LOW.
#define STATUS_BLINK_MS  120
uint32_t statusLedOffAt = 0;
int      statusLedPin   = -1;

void blinkStatus(int pin) {
  statusLedPin = pin;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  statusLedOffAt = millis() + STATUS_BLINK_MS;
}

void updateStatusLed() {
  if (statusLedOffAt && (int32_t)(millis() - statusLedOffAt) >= 0) {
    statusLedOffAt = 0;
    digitalWrite(statusLedPin, HIGH);
  }
}

// ---------------------------------------------------------------------------

float smoothedLevel  = 0.0f;  // 0..MAX_LEVEL, drives center brightness
float smoothedSpread = 0.0f;  // 0..1, drives the Gaussian's width (sigma)
float lastNormalized = 0.0f;  // last 0..1 volume, before any curve
float smoothedVolume = 0.0f;  // lastNormalized, smoothed - drives the race speed
float lastDb = 0.0f;          // last raw dB reading, exposed for the plotter

// ---------------------------------------------------------------------------
// VOICE-BAND FILTER
// ---------------------------------------------------------------------------
// Only energy inside the speech band counts toward the level: a high-pass
// cuts rumble, footsteps, wind and handling noise (and the PDM's DC offset);
// a low-pass cuts hiss and high-pitched noise. Both are 2nd-order Butterworth
// (12dB/octave) biquads. Note: this selects by FREQUENCY, not by "is it a
// voice" - music or clapping inside this band still lights the mask.
#define VOICE_LOW_HZ    200.0f    // below: rumble / thumps / wind
#define VOICE_HIGH_HZ   3500.0f   // above: hiss / clinks / sibilance

struct Biquad {
  float b0, b1, b2, a1, a2;
  float z1, z2;
  float process(float x) {   // transposed direct form II
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }

  // RBJ band-pass, 0dB peak gain at f0.
  void initBandPass(float f0, float q) {
    float w0 = 2.0f * PI * f0 / SAMPLE_RATE;
    float alpha = sinf(w0) / (2.0f * q);
    float a0 = 1.0f + alpha;
    b0 = alpha / a0;
    b1 = 0.0f;
    b2 = -alpha / a0;
    a1 = -2.0f * cosf(w0) / a0;
    a2 = (1.0f - alpha) / a0;
    z1 = z2 = 0.0f;
  }

  // RBJ Audio EQ Cookbook coefficients, Q = 1/sqrt(2) (Butterworth).
  // A member, not a free function: Arduino's auto-generated prototypes
  // would land above the struct and fail to compile.
  void init(bool highPass, float cutoffHz) {
    float w0 = 2.0f * PI * cutoffHz / SAMPLE_RATE;
    float cw = cosf(w0);
    float alpha = sinf(w0) / (2.0f * 0.70710678f);
    float a0 = 1.0f + alpha;
    if (highPass) {
      b0 = (1.0f + cw) / 2.0f / a0;
      b1 = -(1.0f + cw) / a0;
    } else {
      b0 = (1.0f - cw) / 2.0f / a0;
      b1 = (1.0f - cw) / a0;
    }
    b2 = b0;
    a1 = -2.0f * cw / a0;
    a2 = (1.0f - alpha) / a0;
    z1 = z2 = 0.0f;
  }
};

Biquad voiceHighPass, voiceLowPass;

void resetVoiceFilter() {
  voiceHighPass.init(true,  VOICE_LOW_HZ);
  voiceLowPass.init(false, VOICE_HIGH_HZ);
}

// ---------------------------------------------------------------------------
// VOICE PITCH -> HUE
// ---------------------------------------------------------------------------
// The pitch (fundamental frequency) of the voice picks the hue: low voice ->
// HUE_LOW, high voice -> HUE_HIGH, on a log-frequency scale so every musical
// interval moves the color by the same amount.
//
// Detection is YIN (de Cheveigne & Kawahara 2002) - autocorrelation's more
// robust cousin, far less prone to octave errors. It runs on its OWN signal
// path, not the voice-band one above: that one cuts below 200Hz, which is
// exactly where a speaking voice's fundamental lives.
//
// Consonants like s/sh/f have no pitch; YIN reports them unvoiced and the hue
// just holds its last value.
//
// Calibrate with the plotter: talk at your normal, lowest and highest pitch
// and set PITCH_LOW_HZ / PITCH_HIGH_HZ to the ends of your range - a speaking
// voice only spans about an octave, so a range that's too wide barely moves
// the color.
// Calibrated 2026-09-27 (median / p10-p90): talk 106Hz / 93-127,
// low voice 105 / 94-134, shout 150 / 97-198. The voice rises in pitch as it
// gets louder, so shouting lands near the top of the range on its own.
#define PITCH_LOW_HZ    90.0f    // at/below this -> HUE_LOW
#define PITCH_HIGH_HZ   160.0f   // at/above this -> HUE_HIGH

// Adafruit ColorHSV hue scale: 0 = red, 10923 = yellow, 21845 = green,
// 32768 = cyan, 43690 = blue, 54613 = magenta, 65535 = back to red.
#define HUE_LOW         0        // red for a low voice...
#define HUE_HIGH        43690    // ...through yellow and green to blue for a high one

#define HUE_ALPHA       0.25f    // hue smoothing per mic block (~25ms)

// Search range for the detector itself - wider than the color range, so a
// voice slightly outside it still gets a (clamped) reading, not "unvoiced".
#define PITCH_MIN_HZ    70.0f
#define PITCH_MAX_HZ    500.0f

// Decimated 2x to 8kHz: plenty for a <500Hz fundamental, and a quarter of the
// work. 320 samples = 40ms window, which fits ~3 periods of the lowest pitch.
#define PITCH_RATE      (SAMPLE_RATE / 2)
#define PITCH_BUF       320
#define YIN_THRESHOLD   0.20f    // first dip below this = the period
                                 // (0.15 was too strict for a MEMS mic in a room)
#define YIN_UNVOICED    0.35f    // best dip still above this = no pitch
#define YIN_OCTAVE_SLACK 0.10f   // an earlier dip this close to the deepest one wins

Biquad pitchHighPass, pitchLowPass;   // 60Hz..1000Hz: DC/rumble out, anti-alias for 8kHz
float pitchRing[PITCH_BUF];           // oldest sample at pitchHead
volatile int pitchHead = 0;           // written from the PDM callback
bool  pitchPhase = false;             // decimation toggle

float lastPitchHz = 0.0f;   // last detection (0 = unvoiced), exposed for the plotter
float smoothedHue = 0.0f;   // 0..1 position between HUE_LOW and HUE_HIGH

void resetPitch() {
  pitchHighPass.init(true,  60.0f);
  pitchLowPass.init(false, 1000.0f);
  for (int i = 0; i < PITCH_BUF; i++) pitchRing[i] = 0.0f;
  pitchHead = 0;
  lastPitchHz = 0.0f;
}

void pushPitchSample(float raw) {
  float s = pitchLowPass.process(pitchHighPass.process(raw));
  pitchPhase = !pitchPhase;
  if (!pitchPhase) return;   // keep every 2nd sample
  pitchRing[pitchHead] = s;
  pitchHead = (pitchHead + 1) % PITCH_BUF;
}

// YIN over the last PITCH_BUF samples. Returns Hz, or 0 if unvoiced.
float detectPitch() {
  static float x[PITCH_BUF];
  static float cmnd[PITCH_BUF];   // cumulative mean normalized difference
  noInterrupts();   // the PDM callback writes the ring - take a consistent snapshot
  for (int i = 0; i < PITCH_BUF; i++) x[i] = pitchRing[(pitchHead + i) % PITCH_BUF];
  interrupts();

  const int minLag = (int)(PITCH_RATE / PITCH_MAX_HZ);
  const int maxLag = (int)(PITCH_RATE / PITCH_MIN_HZ);
  const int window = PITCH_BUF - maxLag;

  float runningSum = 0.0f;
  cmnd[0] = 1.0f;
  for (int tau = 1; tau <= maxLag; tau++) {
    float d = 0.0f;
    for (int j = 0; j < window; j++) {
      float diff = x[j] - x[j + tau];
      d += diff * diff;
    }
    runningSum += d;
    cmnd[tau] = (runningSum > 0.0f) ? d * tau / runningSum : 1.0f;
  }

  // First dip under the threshold, walked down to its local minimum. Taking
  // the FIRST one (not the global minimum) is what avoids octave-down errors.
  int best = -1;
  for (int tau = minLag; tau <= maxLag; tau++) {
    if (cmnd[tau] < YIN_THRESHOLD) {
      while (tau + 1 <= maxLag && cmnd[tau + 1] < cmnd[tau]) tau++;
      best = tau;
      break;
    }
  }
  if (best < 0) {
    int globalMin = minLag;
    for (int tau = minLag + 1; tau <= maxLag; tau++) {
      if (cmnd[tau] < cmnd[globalMin]) globalMin = tau;
    }
    if (cmnd[globalMin] > YIN_UNVOICED) return 0.0f;

    // The global minimum is often at 2x the true period (the waveform repeats
    // there too) - calibration read a high voice as 89Hz, half of it. Take the
    // FIRST local minimum that's nearly as deep instead.
    float nearlyAsDeep = cmnd[globalMin] + YIN_OCTAVE_SLACK;
    best = globalMin;
    for (int tau = minLag + 1; tau < globalMin; tau++) {
      if (cmnd[tau] < nearlyAsDeep && cmnd[tau] <= cmnd[tau - 1] && cmnd[tau] <= cmnd[tau + 1]) {
        best = tau;
        break;
      }
    }
  }

  // Parabolic interpolation between lags - at 250Hz one lag step is ~3%.
  float period = best;
  if (best > minLag && best < maxLag) {
    float a = cmnd[best - 1], b = cmnd[best], c = cmnd[best + 1];
    float den = a - 2.0f * b + c;
    if (den > 0.0f) period += 0.5f * (a - c) / den;
  }
  return PITCH_RATE / period;
}

// Median of the last 3 detections drops single-frame octave jumps before
// they reach the color; then the log-frequency position is smoothed.
void updateHue(float hz) {
  static float hist[3] = {0, 0, 0};
  static int   idx = 0, filled = 0;
  if (hz <= 0.0f) return;   // unvoiced: hold the current hue

  hist[idx] = hz;
  idx = (idx + 1) % 3;
  if (filled < 3) filled++;
  float med = hz;
  if (filled == 3) {
    float a = hist[0], b = hist[1], c = hist[2];
    med = fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
  }

  float pos = log2f(med / PITCH_LOW_HZ) / log2f(PITCH_HIGH_HZ / PITCH_LOW_HZ);
  pos = constrain(pos, 0.0f, 1.0f);
  smoothedHue += HUE_ALPHA * (pos - smoothedHue);
}

// ---------------------------------------------------------------------------
// DOUBLE TONGUE-CLICK TRIGGER
// ---------------------------------------------------------------------------
// A tongue click, as the XIAO's mic hears it, is a sharp broadband onset
// followed by a ~1.1kHz resonance that rings for 30-60ms. Claps, knocks and
// consonants are broadband too, but their energy isn't concentrated around
// 1.1kHz. The trigger is TWO clicks PAIR_MIN_MS-PAIR_MAX_MS apart.
//
// Designed and tested offline on 32 recordings from this board (6 clean,
// 12 with music, 6 mid-speech, 8 negatives: speech with k/t/p, claps, knocks,
// taps on the board, music, laughter): 21/24 triggers caught, 0 false
// triggers. The 3 misses were all clicks quiet relative to loud music.
//
// Per 2ms frame, three band energies in dB: the click band, and the bands
// either side of it. Each band also has a background tracker (drops fast,
// rises slowly - it sits under transients), so what an event ADDS to a band
// can be separated from music/noise that was already there.
//   onset: click band >= CLICK_ONSET_DB above its background, and up
//          >= CLICK_RISE_DB within 2 frames (4ms)
//   score: over 6-40ms after the onset (the ring, the onset itself excluded),
//          mean of  (click band - louder neighbour)                 [absolute]
//                 + 0.5 * (same, but rises above background)        [vs noise]
//   pair:  both clicks score >= CLICK_WEAK, at least one >= CLICK_STRONG
#define CLICK_FC         1150.0f
#define CLICK_Q          2.0f
#define CLICK_LO_HZ      450.0f
#define CLICK_HI_HZ      2500.0f
#define CLICK_NB_Q       1.5f
#define CLICK_FRAME      32       // samples = 2ms
#define CLICK_ONSET_DB   12.0f
#define CLICK_RISE_DB    6.0f
#define CLICK_W0         3        // ring window, in frames after the onset
#define CLICK_W1         20
#define CLICK_HOLD       20       // frames (40ms) between onsets
#define CLICK_KA         1.0f
#define CLICK_KR         0.5f
// Tightened after a 60s handling session (putting the mask on/off, setting it
// down): the one false trigger there was a +0.6/+2.3 pair 380ms apart. Real
// double clicks were always 170-320ms apart with a strong click >= 3, and
// 3/1/360 keeps the same 21/24 on the test recordings.
#define CLICK_STRONG     3.0f
#define CLICK_WEAK       1.0f
#define PAIR_MIN_MS      110
#define PAIR_MAX_MS      360

// Added after a 60s talking session into the worn mask: with the mic right at
// the mouth, vowel onsets score as high as clicks (1.1kHz formant), and one
// fast syllable - 5 onsets 50ms apart - paired up into a false trigger. What
// still separates them is HOW HARD the onset jumps above the background:
//   real clicks: loud one 20-46dB, quiet one >= 14dB, and the stretch between
//                them falls back to near background (median <= 5.3dB)
//   that speech: 13-21dB, and it never goes quiet in between (5.9-8.7dB)
// All three must hold. Offline: still 21/24 caught, 0 false triggers across
// all recordings incl. both handling sessions and the talking session.
#define CLICK_MIN_RISE     14.0f   // both clicks: peak rise above background, dB
#define CLICK_STRONG_RISE  20.0f   // the louder click of the pair
#define CLICK_MAX_GAP      6.0f    // median rise between the clicks (after the first one's ring)
#define CLICK_GAP_SKIP     25      // frames (50ms) of the first click's ring to skip
#define TRIGGER_COOLDOWN_MS  1000

// Sample side (PDM callback): filters + per-frame energy -> frame ring.
Biquad clickBand, clickLo, clickHi;
float  clickAcc[3] = {0, 0, 0};
int    clickAccN = 0;
#define CLICK_RING  256           // frames (0.5s) of slack for a slow loop()
float  clickRing[CLICK_RING][3];  // dB: click band, low neighbour, high neighbour
volatile uint32_t clickWritten = 0;
uint32_t clickRead = 0;

void pushClickSample(float raw) {
  float y0 = clickBand.process(raw);
  float y1 = clickLo.process(raw);
  float y2 = clickHi.process(raw);
  clickAcc[0] += y0 * y0;
  clickAcc[1] += y1 * y1;
  clickAcc[2] += y2 * y2;
  if (++clickAccN < CLICK_FRAME) return;
  float* f = clickRing[clickWritten % CLICK_RING];
  for (int k = 0; k < 3; k++) {
    f[k] = 10.0f * log10f(clickAcc[k] / CLICK_FRAME + 1.0f);
    clickAcc[k] = 0.0f;
  }
  clickAccN = 0;
  clickWritten++;
}

// Frame side (loop): background trackers, onset/score, pairing.
struct ClickHist { float b, nb, r, nbr; };
#define CLICK_HIST 256            // frames: W1 look-ahead + the whole pair window for the gap check
ClickHist clickHist[CLICK_HIST];
uint32_t  clickFrames = 0;        // frames processed
float     clickBg[3];
int       clickHold = 0;
bool      clickBgInit = false;

struct ClickEvent { uint32_t frame; float score; float rise; };
ClickEvent recentClicks[4];
int        recentN = 0;
uint32_t   lastTriggerMs = 0;
volatile bool clickTriggered = false;   // consumed by loop()

void resetClick() {
  clickBand.initBandPass(CLICK_FC, CLICK_Q);
  clickLo.initBandPass(CLICK_LO_HZ, CLICK_NB_Q);
  clickHi.initBandPass(CLICK_HI_HZ, CLICK_NB_Q);
  clickAcc[0] = clickAcc[1] = clickAcc[2] = 0.0f;
  clickAccN = 0;
  clickWritten = clickRead = clickFrames = 0;
  clickBgInit = false;
  clickHold = 0;
  recentN = 0;
}

// Median rise-above-background between two clicks (skipping the first one's
// ring). Only runs for candidate pairs, which are rare.
float clickGap(uint32_t f1, uint32_t f2) {
  static float buf[CLICK_HIST];
  uint32_t a = f1 + CLICK_GAP_SKIP, b = f2 - 3;
  if (b <= a || b - a < 3) return -99.0f;
  int n = 0;
  for (uint32_t k = a; k < b && n < CLICK_HIST; k++) buf[n++] = clickHist[k % CLICK_HIST].r;
  std::nth_element(buf, buf + n / 2, buf + n);
  return buf[n / 2];
}

void onClick(uint32_t frame, float score, float rise) {
  if (score < CLICK_WEAK || rise < CLICK_MIN_RISE) return;
  // Any earlier unused click in the window pairs with this one - a stray
  // onset between the two real clicks mustn't break the pair.
  for (int a = 0; a < recentN; a++) {
    ClickEvent& e = recentClicks[a];
    uint32_t dms = (frame - e.frame) * 2;
    if (dms < PAIR_MIN_MS || dms > PAIR_MAX_MS) continue;
    if (fmaxf(e.score, score) < CLICK_STRONG) continue;
    if (fmaxf(e.rise, rise) < CLICK_STRONG_RISE) continue;
    if (clickGap(e.frame, frame) > CLICK_MAX_GAP) continue;
    // A pair consumes every click before it, so a late onset from the same
    // gesture can't pair with an older stray one and fire a second trigger.
    recentN = 0;
    if (millis() - lastTriggerMs >= TRIGGER_COOLDOWN_MS) {
      lastTriggerMs = millis();
      clickTriggered = true;
    }
    return;
  }
  if (recentN == 4) {                         // drop the oldest
    for (int a = 1; a < 4; a++) recentClicks[a - 1] = recentClicks[a];
    recentN = 3;
  }
  recentClicks[recentN++] = {frame, score, rise};
}

void processClickFrames() {
  uint32_t written = clickWritten;
  if (written - clickRead > CLICK_RING) clickRead = written - CLICK_RING;   // loop() fell behind
  while (clickRead < written) {
    float* v = clickRing[clickRead % CLICK_RING];
    clickRead++;
    if (!clickBgInit) {
      for (int k = 0; k < 3; k++) clickBg[k] = v[k];
      clickBgInit = true;
    }
    float r[3];
    for (int k = 0; k < 3; k++) {
      r[k] = v[k] - clickBg[k];
      clickBg[k] += ((v[k] > clickBg[k]) ? 0.02f : 0.3f) * r[k];
    }
    clickHist[clickFrames % CLICK_HIST] = {v[0], fmaxf(v[1], v[2]), r[0], fmaxf(r[1], r[2])};
    clickFrames++;

    // Evaluate the frame CLICK_W1-1 back, now that its whole ring window is in.
    if (clickFrames < CLICK_W1 + 2) continue;
    uint32_t i = clickFrames - CLICK_W1;
    if (clickHold > 0) { clickHold--; continue; }
    const ClickHist& h  = clickHist[i % CLICK_HIST];
    const ClickHist& h2 = clickHist[(i - 2) % CLICK_HIST];
    if (h.r < CLICK_ONSET_DB || h.b - h2.b < CLICK_RISE_DB) continue;

    float s = 0.0f;
    for (uint32_t k = i + CLICK_W0; k < i + CLICK_W1; k++) {
      const ClickHist& w = clickHist[k % CLICK_HIST];
      s += CLICK_KA * (w.b - w.nb) + CLICK_KR * (w.r - w.nbr);
    }
    float score = s / (CLICK_W1 - CLICK_W0);
    float rise = h.r;                               // peak rise above background, 0-16ms
    for (uint32_t k = i + 1; k < i + 8; k++) rise = fmaxf(rise, clickHist[k % CLICK_HIST].r);
    float ev[2] = {score, rise};
    streamEvent('C', i * CLICK_FRAME, ev, 2);
    onClick(i, score, rise);
    clickHold = CLICK_HOLD;
  }
}

// ---------------------------------------------------------------------------
// DOUBLE-TAP TRIGGER (accelerometer)
// ---------------------------------------------------------------------------
// Two taps on the device switch the mode. Designed on two guided recordings
// (833Hz accel from the FIFO, mask worn): 10+10 double taps standing, 8 while
// dancing, plus a minute+ of dancing/walking/nodding and putting the mask
// on/off/down without taps. Every double tap was caught, 0 false triggers.
//
// Signal: |accel| after a 2nd-order 30Hz high-pass per axis - gravity and
// body motion are slow, a tap is a sharp few-ms jolt.
//   real taps:            10-26g (light ones as low as ~2-5g for the 2nd tap)
//   dancing / walking:    <= 2.2g
//   on / off / set down:  <= 2.2g
// A tap = the signal crossing TAP_WEAK_G; its strength = the peak over the
// next TAP_REFRACT_MS (which also swallows the tap's own rebound ~70-100ms
// later). A pair = two taps TAP_PAIR_MIN..MAX_MS apart, one >= TAP_STRONG_G
// (2x anything movement produced) and the other >= TAP_WEAK_G.
// Taps are counted as a sequence (each one TAP_PAIR_MIN..MAX_MS after the
// last); TAP_PAIR_MAX_MS of quiet after the last tap ends it. 2 taps = next
// mode, 3 = off, 4 = dim on/off, 5+ = sound floor on/off. So a double tap
// fires ~0.5s after the second tap instead of right on it.
#define TAP_FS            905.0f   // the IMU's real 833Hz-nominal FIFO rate, measured
#define TAP_HP_HZ         30.0f
#define TAP_STRONG_G      4.5f
#define TAP_WEAK_G        2.0f
#define TAP_REFRACT_MS    100
#define TAP_PAIR_MIN_MS   120
#define TAP_PAIR_MAX_MS   500
#define TAP_COOLDOWN_MS   1000

struct TapHP { float b0, b1, b2, a1, a2, z1[3], z2[3]; };
TapHP    tapHP;
uint32_t tapSamples   = 0;      // FIFO samples processed since resetTap()
int      tapInPeak    = 0;      // samples left in the current tap's window
float    tapPeak      = 0.0f;
uint32_t tapPeakStart = 0;
int      tapSeqCount  = 0;      // taps in the current sequence
uint32_t tapLastStart = 0;      // start of the sequence's latest tap
bool     tapSeqStrong = false;  // any tap in the sequence >= TAP_STRONG_G
volatile int  tapDebugCount = 0;   // loop() prints "TAPS n" for each completed sequence
volatile int  tapCommand = 0;   // taps in the last completed sequence (2+), consumed by loop()
uint32_t tapLastTrig  = 0;
bool     tapTrigEver  = false;

void resetTap() {
  float w0 = 2.0f * PI * TAP_HP_HZ / TAP_FS;
  float cw = cosf(w0), al = sinf(w0) / (2.0f * 0.70710678f), a0 = 1.0f + al;
  tapHP.b0 = (1.0f + cw) / 2.0f / a0;
  tapHP.b1 = -(1.0f + cw) / a0;
  tapHP.b2 = tapHP.b0;
  tapHP.a1 = -2.0f * cw / a0;
  tapHP.a2 = (1.0f - al) / a0;
  for (int k = 0; k < 3; k++) tapHP.z1[k] = tapHP.z2[k] = 0.0f;
  tapSamples = 0; tapInPeak = 0; tapSeqCount = 0; tapTrigEver = false;
}

inline uint32_t tapMs(uint32_t samples) { return (uint32_t)(samples * 1000.0f / TAP_FS); }

void tapSample(const int16_t* raw) {
  float m2 = 0.0f;
  for (int k = 0; k < 3; k++) {
    float x = raw[k] * ACCEL_RAW_TO_G;
    float y = tapHP.b0 * x + tapHP.z1[k];
    tapHP.z1[k] = tapHP.b1 * x - tapHP.a1 * y + tapHP.z2[k];
    tapHP.z2[k] = tapHP.b2 * x - tapHP.a2 * y;
    m2 += y * y;
  }
  uint32_t n = tapSamples++;
  if (n < (uint32_t)(TAP_FS / 4)) return;          // let the filter settle after (re)start
  float m = sqrtf(m2);

  if (tapInPeak > 0) {
    if (m > tapPeak) tapPeak = m;
    if (--tapInPeak > 0) return;
    // the tap is over: extend the sequence, or start a new one with it
    uint32_t dms = tapMs(tapPeakStart - tapLastStart);
    if (tapSeqCount > 0 && dms >= TAP_PAIR_MIN_MS && dms <= TAP_PAIR_MAX_MS) {
      tapSeqCount++;
      tapSeqStrong = tapSeqStrong || tapPeak >= TAP_STRONG_G;
    } else {
      tapSeqCount = 1;
      tapSeqStrong = tapPeak >= TAP_STRONG_G;
    }
    tapLastStart = tapPeakStart;
    return;
  } else if (tapSeqCount > 0 && tapMs(n - tapLastStart) > TAP_PAIR_MAX_MS) {
    // no follow-up tap in time: the sequence is complete
    if (tapSeqCount >= 2 && tapSeqStrong &&
        (!tapTrigEver || tapMs(tapLastStart - tapLastTrig) >= TAP_COOLDOWN_MS)) {
      tapLastTrig = tapLastStart;
      tapTrigEver = true;
      tapCommand = tapSeqCount;
      tapDebugCount = tapSeqCount;
    }
    tapSeqCount = 0;
  }
  if (tapInPeak == 0 && m >= TAP_WEAK_G) {
    tapInPeak = (int)(TAP_REFRACT_MS * TAP_FS / 1000.0f);
    tapPeak = m;
    tapPeakStart = n;
  }
}

// Drains the IMU FIFO every loop: every sample goes through the tap detector,
// and while streaming also out to the PC as 'F' packets.
void serviceAccel() {
  if (!imuOk) return;
  static int16_t acc[3 * 64];
  bool over;
  int n;
  while ((n = fifoRead(acc, 64, &over)) > 0) {
    for (int k = 0; k < n; k++) tapSample(&acc[3 * k]);
    if (streaming) {
      // uint32 mic index, uint8 overrun, uint8 n, int16 xyz[n]
      uint8_t hdr[6];
      memcpy(hdr, (const void*)&micSampleCount, 4);
      hdr[4] = over;
      hdr[5] = (uint8_t)n;
      streamPacket('F', hdr, 6, acc, n * 6);
    }
    if (n < 64) break;
  }
}

// Computes RMS of the voice-band-filtered signal over whatever PDM samples
// have arrived since the last call.
// Returns -1 if no new samples are ready yet (caller should skip this tick).
float readMicRMS() {
  if (pdmSamplesRead <= 0) return -1.0f;

  noInterrupts();
  int count = pdmSamplesRead;
  pdmSamplesRead = 0;
  interrupts();

  double sumSquares = 0.0;
  for (int i = 0; i < count; i++) {
    float sample = voiceLowPass.process(voiceHighPass.process((float)pdmBuffer[i]));
    sumSquares += (double)sample * sample;
  }
  double meanSquare = sumSquares / count;
  return (float)sqrt(meanSquare);
}

// Converts RMS amplitude to a 0..1 level using NOISE_FLOOR_DB..LOUD_DB,
// then applies the compressive curve.
float rmsToLevel(float rms) {
  float db = (rms > 1.0f) ? 20.0f * log10f(rms) : 0.0f;
  lastDb = db;   // stashed for the Serial Plotter output in loop()

  float normalized = (db - NOISE_FLOOR_DB) / (LOUD_DB - NOISE_FLOOR_DB);
  normalized = constrain(normalized, 0.0f, 1.0f);
  lastNormalized = normalized;   // the spread uses its own curve, see SPREAD_EXPONENT

  float curved = powf(normalized, CURVE_EXPONENT);
  return curved * MAX_LEVEL;
}

// Smooth the raw level with an asymmetric attack/decay envelope follower.
float smoothLevel(float target, float current) {
  float alpha = (target > current) ? ATTACK_ALPHA : DECAY_ALPHA;
  return current + alpha * (target - current);
}

// ---------------------------------------------------------------------------
// X-CENTER GAUSSIAN EFFECT
// ---------------------------------------------------------------------------
// The X's center: this LED on each strip. Both sit at the same physical
// spot. Light starts there and spreads outward along both strips with a
// Gaussian falloff.
// Strip-local LED positions; may be x.5 when the crossing point falls
// between two LEDs. Tuned live with center_tuner.py ('K' command).
#define STRIP1_CENTER  29.0f
#define STRIP2_CENTER  31.0f
float strip1Center = STRIP1_CENTER;
float strip2Center = STRIP2_CENTER;

// Signed distance (in LEDs, along its own strip) from a virtual pixel to the
// X's center: negative = toward the strip's LED 0, positive = toward its end.
inline float centerOffset(int i) {
  return (i < NUM_LEDS1) ? (float)i - strip1Center : (float)(i - NUM_LEDS1) - strip2Center;
}

// Gaussian width (sigma, in pixels) grows with volume: quiet speech lights
// only a small glow at the center, loud speech spreads down all four arms.
#define SIGMA_MIN    3.0f    // at the quietest audible level
#define SIGMA_MAX    45.0f   // at LOUD_DB

// The spread has its own curve, separate from the center's CURVE_EXPONENT:
// the center should react to quiet speech, while the edges only light up
// when it gets loud. >1 = spread stays tight until the volume is high.
#define SPREAD_EXPONENT  3.0f

// The eye sees LED brightness roughly logarithmically: 20% PWM already looks
// nearly full, and a Gaussian's tails look as bright as its peak, flattening
// the shape. Levels above are in PERCEIVED brightness; this converts each
// pixel to PWM duty (duty = perceived^gamma) so the falloff reads as a falloff.
#define LED_GAMMA    2.2f

float distToCenter[NUM_LEDS];     // |centerOffset()|, cached

// ---------------------------------------------------------------------------
// MODES (cycled by the double tongue-click trigger)
// ---------------------------------------------------------------------------
enum Mode { MODE_TALK, MODE_PARTY, MODE_RACE, MODE_CONVERGE, MODE_COUNT };
const char* MODE_NAMES[MODE_COUNT] = {"X mode", "party", "led race", "X converging"};
uint8_t mode = MODE_TALK;

uint32_t triggerFlashAt = 0;          // 0 = no flash running
uint32_t triggerFlashRGB = 0xFFFFFF;  // white = next mode; sound floor: green = on, red = off
// Wake-up: the whole X breathes pink WAKE_ANIM_PULSES times, softly, then
// the current mode takes over.
uint32_t wakeAnimAt = 0;              // 0 = not running
#define  WAKE_ANIM_MS        2400
#define  WAKE_ANIM_PULSES    2
#define  WAKE_ANIM_LEVEL     0.45f    // perceived brightness at the top of a breath
#define  WAKE_ANIM_RGB       0xFF2A8C   // pink

// Dim mode (4 taps, as on the Somcha band): every channel is scaled by
// DIM_FACTOR at the very end, after the current limiter, so it really is
// that fraction of the light and current. Not saved: a power cycle starts
// bright again (sleep keeps it).
#define  DIM_FACTOR          0.3f
bool     dimMode = false;

// Manual off (3 taps): the current picture fades out over OFF_FADE_MS, then
// the mask goes to sleep. A double tap wakes it, like the idle sleep.
#define  OFF_FADE_MS         800
uint32_t offFadeAt = 0;               // 0 = not fading
#define  TRIGGER_FLASH_MS    150
#define  TRIGGER_FLASH_DUTY  0.15f    // whole strip, white - ~340mA, under the limiter

// Overall brightness of every effect, in PERCEIVED terms (applied through the
// gamma curve, like everything else): 0.5 = looks half as bright = PWM x0.22.
#define GLOBAL_LEVEL  0.5f

// Every mode fills a PWM duty (0..1) and a full-scale color per pixel; one
// shared pass then applies the current limiter and pushes them to the strip.
float    pixDuty[NUM_LEDS];
uint32_t pixColor[NUM_LEDS];

void showPixels() {
  float load = 0.0f;   // sum of duty * (R+G+B) over the strip
  for (int i = 0; i < NUM_LEDS; i++) {
    uint32_t c = pixColor[i];
    load += pixDuty[i] * (((c >> 16) & 0xFF) + ((c >> 8) & 0xFF) + (c & 0xFF));
  }
  const float master = powf(GLOBAL_LEVEL, LED_GAMMA);
  float k = powerLimitFactor(load * master, 1) * master;
  if (dimMode) k *= DIM_FACTOR;
  if (offFadeAt) k *= fmaxf(0.0f, 1.0f - (millis() - offFadeAt) / (float)OFF_FADE_MS);
  for (int i = 0; i < NUM_LEDS; i++) {
    uint32_t c = pixColor[i];
    float w = pixDuty[i] * k;
    uint8_t r = ((c >> 16) & 0xFF) * w, g = ((c >> 8) & 0xFF) * w, b = (c & 0xFF) * w;
    if (i < NUM_LEDS1) strip.setPixelColor(i, r, g, b);
    else               strip2.setPixelColor(i - NUM_LEDS1, r, g, b);
  }
  strip.show();
  strip2.show();
}

// The voice's current color: hue from pitch (or the fixed color).
uint32_t voiceColor() {
  if (!COLOR_BY_PITCH) return strip.Color(LED_R, LED_G, LED_B);
  uint16_t hue = (uint16_t)(long)(HUE_LOW + smoothedHue * ((float)HUE_HIGH - HUE_LOW));
  return strip.ColorHSV(hue, 255, 255);
}

// --- MODE 1: "X mode" --------------------------------------------------------
// Brightness at the X center follows the voice envelope; each pixel gets
// level * exp(-d^2 / 2sigma^2), where d is its distance (in pixels, along the
// strip) to the nearer of the two center pixels. Color follows pitch.
void renderTalk() {
  float sigma = SIGMA_MIN + smoothedSpread * (SIGMA_MAX - SIGMA_MIN);
  float inv2Sigma2 = 1.0f / (2.0f * sigma * sigma);
  uint32_t color = voiceColor();
  for (int i = 0; i < NUM_LEDS; i++) {
    float d = distToCenter[i];
    pixDuty[i] = powf(smoothedLevel * expf(-d * d * inv2Sigma2), LED_GAMMA);
    pixColor[i] = color;
  }
}

// --- MODE 2: "party" ---------------------------------------------------------
// A rainbow flows along the strips at a constant speed; on top of it, the
// brightness of every LED is shaped by Perlin noise - smooth random blobs of
// light and dark that slowly morph. The voice plays the noise:
//   volume -> DEPTH: silence = an almost steady rainbow, loud = deep contrast
//             between bright blobs and near-dark gaps
//   pitch  -> GRAIN: a low voice gives wide blobs, a high voice small dense
//             grains. Grain is measured from the X's center, so a change in
//             pitch looks like the pattern zooming in/out around the center.
// Both are smoothed, so the pattern flows instead of jumping with every
// syllable. Both strips show the same pattern (position = distance along the
// strip from its own center LED).
#define PARTY_HUE_CYCLE_S   10.0f   // seconds for the mask to go once around the hue wheel
#define PARTY_RAINBOW_LEDS  60.0f   // LEDs for one full rainbow (each strip spans ~60)
#define PARTY_LEVEL         0.5f    // perceived brightness at the brightest blobs
// First tuning (depth 0.15-0.95 linear in volume, blobs 24->4 LEDs, rate 0.5)
// was barely visible: one blob per ~30-LED arm, half depth at normal speech.
// Second round (0.35-1.0, 10->3 LEDs, rate 1.2): better but asked for more.
#define PARTY_DEPTH_QUIET   0.6f    // noise depth in silence (0 = flat, 1 = down to dark)
#define PARTY_DEPTH_LOUD    1.0f    // noise depth at LOUD_DB (sqrt curve: deep already when talking)
#define PARTY_GRAIN_LOW     7.0f    // blob size in LEDs for a low voice
#define PARTY_GRAIN_HIGH    2.5f    // blob size in LEDs for a high voice
#define PARTY_NOISE_RATE    2.0f    // how fast the noise morphs (noise units / second)
#define PARTY_CONTRAST      1.3f    // noise stretch: >1 clips more of it to full dark / full bright
#define PARTY_SMOOTH        0.15f   // per-frame smoothing of depth and grain

// Perlin gradient noise on a 256-period lattice. The permutation is shuffled
// once at boot with a fixed seed, so the "texture" is the same every time.
uint8_t perlinPerm[256];

void initPerlin() {
  for (int i = 0; i < 256; i++) perlinPerm[i] = i;
  uint32_t r = 0x1234567u;
  for (int i = 255; i > 0; i--) {
    r = r * 1664525u + 1013904223u;
    int j = (r >> 8) % (i + 1);
    uint8_t t = perlinPerm[i]; perlinPerm[i] = perlinPerm[j]; perlinPerm[j] = t;
  }
}

inline uint8_t pp(int i) { return perlinPerm[i & 255]; }
inline float perlinFade(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }
inline float perlinGrad(uint8_t h, float x, float y) {
  switch (h & 7) {
    case 0: return  x + y;  case 1: return -x + y;  case 2: return  x - y;  case 3: return -x - y;
    case 4: return  x;      case 5: return -x;      case 6: return  y;      default: return -y;
  }
}

// 2D Perlin noise, roughly -1..1.
float perlin2(float x, float y) {
  float fx = floorf(x), fy = floorf(y);
  int X = (int)fx, Y = (int)fy;
  x -= fx; y -= fy;
  float u = perlinFade(x), v = perlinFade(y);
  float n00 = perlinGrad(pp(pp(X) + Y),         x,        y);
  float n10 = perlinGrad(pp(pp(X + 1) + Y),     x - 1.0f, y);
  float n01 = perlinGrad(pp(pp(X) + Y + 1),     x,        y - 1.0f);
  float n11 = perlinGrad(pp(pp(X + 1) + Y + 1), x - 1.0f, y - 1.0f);
  float nx0 = n00 + u * (n10 - n00);
  float nx1 = n01 + u * (n11 - n01);
  return nx0 + v * (nx1 - nx0);
}

float    partyHue    = 0.0f;   // cycles, 0..1
float    partyNoiseT = 0.0f;   // noise time, wrapped at the lattice period (seamless)
float    partyDepth  = PARTY_DEPTH_QUIET;
float    partyGrain  = PARTY_GRAIN_LOW;
uint32_t partyLastMs = 0;

void renderParty() {
  uint32_t now = millis();
  float dt = partyLastMs ? (now - partyLastMs) / 1000.0f : 0.0f;
  partyLastMs = now;
  partyHue += dt / PARTY_HUE_CYCLE_S;                 // constant flow speed
  partyHue -= floorf(partyHue);
  partyNoiseT += dt * PARTY_NOISE_RATE;
  if (partyNoiseT >= 256.0f) partyNoiseT -= 256.0f;

  // voice -> noise: depth from volume, grain from pitch (log scale, like the hue)
  float depthTarget = PARTY_DEPTH_QUIET + (PARTY_DEPTH_LOUD - PARTY_DEPTH_QUIET) * sqrtf(smoothedVolume);
  float grainTarget = PARTY_GRAIN_LOW * powf(PARTY_GRAIN_HIGH / PARTY_GRAIN_LOW, smoothedHue);
  partyDepth += PARTY_SMOOTH * (depthTarget - partyDepth);
  partyGrain += PARTY_SMOOTH * (grainTarget - partyGrain);

  for (int i = 0; i < NUM_LEDS; i++) {
    float u = centerOffset(i);
    float h = partyHue + u / PARTY_RAINBOW_LEDS;
    h -= floorf(h);
    // two octaves for a less "regular" texture
    float x = u / partyGrain;
    float n = perlin2(x, partyNoiseT) + 0.5f * perlin2(2.0f * x + 37.0f, 2.0f * partyNoiseT + 11.0f);
    float n01 = constrain(0.5f + 0.75f * PARTY_CONTRAST * n, 0.0f, 1.0f);
    n01 *= n01;                                          // darker, sharper gaps between the blobs
    float p = PARTY_LEVEL * (1.0f - partyDepth * (1.0f - n01));
    pixDuty[i] = powf(p, LED_GAMMA);
    pixColor[i] = strip.ColorHSV((uint16_t)(h * 65535.0f), 255, 255);
  }
}

// --- MODE 3: "led race" ------------------------------------------------------
// RACE_GROUPS small Gaussian blobs (~5 LEDs each), evenly spaced, run along
// the strip together, colored by the voice's pitch. Their speed follows the
// voice volume: RACE_MIN_LPS in silence, up to RACE_MAX_LPS at LOUD_DB, on an
// EXPONENTIAL curve - every dB multiplies the speed by the same factor. Speed
// is perceived as a ratio, so this makes each step in volume equally visible
// across the whole range. (A sqrt curve put normal talk at ~170 and a shout at
// 250: only 1.5x apart, which reads as "the same speed".) Positions
// are fractional, so the blobs glide between pixels instead of stepping.
// Wraps from the end of the strip back to the start.
// Phase offset of strip 2's groups, in LEDs. The groups repeat every 30 LEDs,
// so only offsets 0-29 mean anything, and 15 (half a period) is the most
// different: when a group on one strip reaches the X's center, the other
// strip's groups are as far from it as they can be. (29 would look like 1.)
#define RACE_STRIP2_PHASE  15
#define RACE_GROUPS     4       // evenly spaced: 120 virtual LEDs / 4 = 30 LEDs apart
                                // (was 8 on the old 240-LED single strip)
#define RACE_SIGMA      1.0f    // pixels: +-2 sigma = the 5-LED group
#define RACE_LEVEL      0.7f    // perceived brightness at each blob's center
#define RACE_MIN_LPS    10.0f   // LEDs/second in silence
#define RACE_MAX_LPS    250.0f  // LEDs/second at LOUD_DB
#define RACE_CURVE      0.7f    // <1 lifts the middle (normal talk: ~44 -> ~65 LEDs/s)
                                // while silence and LOUD_DB stay at MIN / MAX

float    racePos    = 0.0f;     // position of the first group; the rest follow at fixed spacing
uint32_t raceLastMs = 0;

// The race path: strip 1 from LED 0 to its end, then strip 2 BACKWARD, from
// its last LED to LED 0, then around again.
inline int racePathPixel(int q) {
  return (q < NUM_LEDS1) ? q : NUM_LEDS1 + (NUM_LEDS2 - 1 - (q - NUM_LEDS1));
}

void renderRace() {
  uint32_t now = millis();
  float dt = raceLastMs ? (now - raceLastMs) / 1000.0f : 0.0f;
  raceLastMs = now;
  float speed = RACE_MIN_LPS * powf(RACE_MAX_LPS / RACE_MIN_LPS, powf(smoothedVolume, RACE_CURVE));
  racePos += speed * dt;
  racePos -= NUM_LEDS * floorf(racePos / NUM_LEDS);

  uint32_t color = voiceColor();
  const float spacing = (float)NUM_LEDS / RACE_GROUPS;
  const float inv2Sigma2 = 1.0f / (2.0f * RACE_SIGMA * RACE_SIGMA);
  for (int q = 0; q < NUM_LEDS; q++) {
    // q = step along the race path; distance to the nearest group: fold it
    // into one spacing period
    float qq = (q < NUM_LEDS1) ? q : q + RACE_STRIP2_PHASE;   // strip 2 runs out of phase
    float d = fmodf(qq - racePos + 2 * NUM_LEDS, spacing);
    d = fminf(d, spacing - d);
    int i = racePathPixel(q);
    pixDuty[i] = (d < 4.0f * RACE_SIGMA) ? powf(RACE_LEVEL * expf(-d * d * inv2Sigma2), LED_GAMMA) : 0.0f;
    pixColor[i] = color;
  }
}

// --- MODE 4: "X converging" --------------------------------------------------
// While talking, comets stream in along all four arms of the X and vanish at
// its center. Each strip is one diagonal through the center, so the four
// arms are: each end of each strip -> that strip's center LED. One wave launches a comet on every arm at once, CONV_START_DIST
// LEDs out from the center, so all four arrive together. (If CONV_START_DIST
// is raised past an arm's length, that arm is shorter: CONV_SYNC_ARRIVAL 1
// scales each arm so all four still arrive together, 0 = same LEDs/second.)
#define CONV_SYNC_ARRIVAL 1
//
// Each comet is a HALF Gaussian: the leading LED at full brightness, a
// fading tail behind it, nothing in front. Its color is the voice's pitch at
// the moment it launched, on a red-centered palette: normal speech red, a low
// voice orange, a high voice purple. Speed follows volume like the race
// (exponential curve); in silence no new comets launch and the ones already
// flying finish at CONV_MIN_LPS.
#define CONV_MIN_LPS      22.0f    // LEDs/s in silence
#define CONV_MAX_LPS      150.0f   // LEDs/s at LOUD_DB
#define CONV_SPACING      14.0f    // LEDs of travel between launches while talking
#define CONV_TAIL_SIGMA   3.0f     // tail length (half Gaussian sigma, LEDs)
#define CONV_LEVEL        0.8f     // perceived brightness of each comet's head
#define CONV_TALK_VOLUME  0.03f    // smoothedVolume above this = talking -> launch
#define CONV_START_DIST   30       // comets launch this many LEDs out from the center

// The rest of the strip (outside the comet paths) glows with the voice: same
// pitch palette, brightness proportional to the volume.
#define CONV_BG_LEVEL     0.6f     // perceived brightness of the glow at LOUD_DB

// Palette by pitch (Hz), all hues on the red side of the wheel, and a
// continuous gradient - every change in pitch shifts the hue a little, no
// flat bands: red exactly at CONV_RED_HZ, sliding (linearly in log-frequency)
// toward orange below it and toward purple above it. Pure orange / purple
// only at the ends of the range.
// From the calibration: normal talk ~106Hz (93-127), shouting rises to 150+.
#define CONV_ORANGE_HZ    90.0f
#define CONV_RED_HZ       108.0f
#define CONV_PURPLE_HZ    155.0f
// >1 keeps the hue near red for longer: still a continuous gradient, but it
// leaves red slowly around normal speech and speeds up toward the extremes.
// (At 1.0 normal speech already read as mostly pink/purple/orange.)
#define CONV_RED_BIAS     2.5f
#define CONV_HUE_ORANGE   4500     // ColorHSV hue units: red = 0 / 65536
#define CONV_HUE_PURPLE   (-15500) // below red: 65536 - 15500 = purple

// The four arms = the two sides of each strip's center. Everything is in
// distance-from-center, so a center between two LEDs (x.5) and arms of
// different lengths just work: a comet's head starts armLen out and moves to 0.
// armLen = min(CONV_START_DIST, distance from the center to that strip end).
float convArmLen[4];               // [strip*2 + side], side 0 = toward LED 0, 1 = toward the end
float convRefLen = 1.0f;           // shortest arm - comet positions are in its units (sync mode)
float convMaxLen = 1.0f;           // longest arm

#define CONV_MAX_COMETS 24
struct Comet { float pos; uint16_t hue; };
Comet    comets[CONV_MAX_COMETS];
int      cometCount     = 0;
float    convSinceLaunch = CONV_SPACING;   // launch immediately when talking starts
uint32_t convLastMs     = 0;
bool     convDemo       = false;   // 'W': launch comets without talking, for tuning the center
#define  CONV_DEMO_LPS    40.0f

inline int convArmOf(int i) {
  return ((i < NUM_LEDS1) ? 0 : 2) + (centerOffset(i) < 0.0f ? 0 : 1);
}

void initConvergeArms() {
  float len[4] = {strip1Center, (NUM_LEDS1 - 1) - strip1Center,
                  strip2Center, (NUM_LEDS2 - 1) - strip2Center};
  convRefLen = 1e9f; convMaxLen = 0.0f;
  for (int a = 0; a < 4; a++) {
    convArmLen[a] = fminf(len[a], (float)CONV_START_DIST);
    convRefLen = fminf(convRefLen, convArmLen[a]);
    convMaxLen = fmaxf(convMaxLen, convArmLen[a]);
  }
}

// Distances / arms depend on the centers: recompute whenever they change.
void recomputeLayout() {
  for (int i = 0; i < NUM_LEDS; i++) distToCenter[i] = fabsf(centerOffset(i));
  initConvergeArms();
}

// Pitch -> hue: continuous orange <- red -> purple gradient in log-frequency.
uint16_t convergeHue() {
  float hz = PITCH_LOW_HZ * powf(PITCH_HIGH_HZ / PITCH_LOW_HZ, smoothedHue);   // back from 0..1 to Hz
  float h;
  if (hz < CONV_RED_HZ) {
    float u = log2f(CONV_RED_HZ / hz) / log2f(CONV_RED_HZ / CONV_ORANGE_HZ);
    h = CONV_HUE_ORANGE * powf(constrain(u, 0.0f, 1.0f), CONV_RED_BIAS);
  } else {
    float u = log2f(hz / CONV_RED_HZ) / log2f(CONV_PURPLE_HZ / CONV_RED_HZ);
    h = CONV_HUE_PURPLE * powf(constrain(u, 0.0f, 1.0f), CONV_RED_BIAS);
  }
  return (uint16_t)(long)lroundf(h);               // negative wraps to the purple side
}

void renderConverge() {
  uint32_t now = millis();
  float dt = convLastMs ? (now - convLastMs) / 1000.0f : 0.0f;
  convLastMs = now;
  float speed = convDemo ? CONV_DEMO_LPS
                         : CONV_MIN_LPS * powf(CONV_MAX_LPS / CONV_MIN_LPS, powf(smoothedVolume, RACE_CURVE));
  float step = speed * dt;

  // move, and drop comets whose tail has fully entered the center
  float life = (CONV_SYNC_ARRIVAL ? convRefLen : convMaxLen) + 4.0f * CONV_TAIL_SIGMA;
  int keep = 0;
  for (int c = 0; c < cometCount; c++) {
    comets[c].pos += step;
    if (comets[c].pos < life) comets[keep++] = comets[c];
  }
  cometCount = keep;

  // launch a new wave every CONV_SPACING LEDs of travel while talking
  convSinceLaunch += step;
  bool talking = convDemo || smoothedVolume > CONV_TALK_VOLUME;
  if (talking && convSinceLaunch >= CONV_SPACING && cometCount < CONV_MAX_COMETS) {
    comets[cometCount++] = {0.0f, convergeHue()};
    convSinceLaunch = 0.0f;
  }

  // beyond the comet paths: a glow that follows the voice
  uint32_t glowColor = strip.ColorHSV(convergeHue(), 255, 255);
  float glowDuty = powf(CONV_BG_LEVEL * smoothedVolume, LED_GAMMA);
  const float inv2s2 = 1.0f / (2.0f * CONV_TAIL_SIGMA * CONV_TAIL_SIGMA);
  for (int i = 0; i < NUM_LEDS; i++) {
    float dc = distToCenter[i];
    float L = convArmLen[convArmOf(i)];
    pixColor[i] = glowColor;
    pixDuty[i] = (dc > L + 0.5f) ? glowDuty : 0.0f;
    if (dc > L + 0.5f) continue;
    for (int c = 0; c < cometCount; c++) {
      // head's distance from the center on this arm (sync: every arm's comet
      // arrives at the same moment, whatever the arm's length)
      float hd = CONV_SYNC_ARRIVAL ? L - comets[c].pos * L / convRefLen : L - comets[c].pos;
      float k = dc - hd;                                   // >0 = behind the head (farther out)
      if (k > 4.0f * CONV_TAIL_SIGMA || k < -1.0f) continue;
      float bri = (k >= 0.0f) ? expf(-k * k * inv2s2)      // tail
                              : 1.0f + k;                  // sub-pixel lead-in, keeps motion smooth
      float duty = powf(CONV_LEVEL * bri, LED_GAMMA);
      if (duty > pixDuty[i]) { pixDuty[i] = duty; pixColor[i] = strip.ColorHSV(comets[c].hue, 255, 255); }
    }
  }
}

// LED finder (development tool, driven by led_finder.py): "L<strip>,<index>"
// lights exactly one LED white and freezes the effects; "L0,0" goes back to
// normal. Used to map which LED numbers sit where on the mask.
int findStrip = 0;       // 0 = off, 1 / 2 = which strip
int findIndex = 0;

void renderFinder() {
  strip.clear();
  strip2.clear();
  Adafruit_NeoPixel& s = (findStrip == 1) ? strip : strip2;
  if (findIndex >= 0 && findIndex < (int)s.numPixels()) s.setPixelColor(findIndex, 60, 60, 60);
  strip.show();
  strip2.show();
}

// Layout test ('C'): through the SAME virtual-strip path the effects use -
// red = within 3 LEDs of each strip's center, blue = the last 3 LEDs at each
// end of each strip. Shown for 15s.
uint32_t layoutTestUntil = 0;

void renderLayoutTest() {
  for (int i = 0; i < NUM_LEDS; i++) {
    int pos = (i < NUM_LEDS1) ? i : i - NUM_LEDS1;
    int len = (i < NUM_LEDS1) ? NUM_LEDS1 : NUM_LEDS2;
    bool center = distToCenter[i] <= 3;
    bool end = pos < 3 || pos >= len - 3;
    pixDuty[i] = (center || end) ? 0.15f : 0.0f;
    pixColor[i] = center ? strip.Color(255, 0, 0) : strip.Color(0, 0, 255);
  }
  showPixels();
}

void renderEffect() {
  if (findStrip) { renderFinder(); return; }
  if (layoutTestUntil && (int32_t)(millis() - layoutTestUntil) < 0) { renderLayoutTest(); return; }
  layoutTestUntil = 0;
  // Confirmation that the trigger was heard: a short white flash of the whole strip.
  if (triggerFlashAt && millis() - triggerFlashAt < TRIGGER_FLASH_MS) {
    float k = TRIGGER_FLASH_DUTY * powf(GLOBAL_LEVEL, LED_GAMMA) * (dimMode ? DIM_FACTOR : 1.0f);
    uint8_t r = ((triggerFlashRGB >> 16) & 0xFF) * k, g = ((triggerFlashRGB >> 8) & 0xFF) * k, b = (triggerFlashRGB & 0xFF) * k;
    strip.fill(strip.Color(r, g, b));
    strip2.fill(strip2.Color(r, g, b));
    strip.show();
    strip2.show();
    return;
  }
  triggerFlashAt = 0;
  if (wakeAnimAt && millis() - wakeAnimAt < WAKE_ANIM_MS) {
    float t = (millis() - wakeAnimAt) / (float)WAKE_ANIM_MS;
    float level = WAKE_ANIM_LEVEL * (0.5f - 0.5f * cosf(2.0f * PI * WAKE_ANIM_PULSES * t));
    float duty = powf(level, LED_GAMMA);
    for (int i = 0; i < NUM_LEDS; i++) { pixDuty[i] = duty; pixColor[i] = WAKE_ANIM_RGB; }
    showPixels();
    return;
  }
  wakeAnimAt = 0;

  switch (mode) {
    case MODE_PARTY: renderParty(); break;
    case MODE_RACE:  renderRace();  break;
    case MODE_CONVERGE: renderConverge(); break;
    default:         renderTalk();  break;
  }
  showPixels();
}

bool startMic() {
  PDM.onReceive(onPDMdata);
  PDM.setGain(MIC_GAIN);
  pdmSamplesRead = 0;
  micSampleCount = 0;
  streamTail = 0;
  resetVoiceFilter();   // no stale filter state from before sleep
  resetPitch();
  resetClick();
  return PDM.begin(1, SAMPLE_RATE);   // mono, 16kHz
}

// With the switch on the ground side, a cut-off strip's ground floats up
// toward V+. A data pin still driven LOW would then sink current from the
// strip through DIN's protection diode, so it goes high-impedance first.
void stripPowerOff() {
  pinMode(LED_PIN, INPUT);
  pinMode(LED2_PIN, INPUT);
  digitalWrite(STRIP_PWR_PIN, LOW);
  digitalWrite(STRIP2_PWR_PIN, LOW);
}

// Safety: if a gate pin is shorted to ground (miswired MOSFET leg, solder
// bridge G-S), driving it HIGH would just push current into the short.
// Check with the weak pull-up first and leave it undriven if it can't rise.
bool gateOn(int pin, const char* name) {
  pinMode(pin, INPUT_PULLUP);
  delay(2);
  if (digitalRead(pin) == LOW) {
    pinMode(pin, INPUT);
    Serial.print(name);
    Serial.println(" POWER PIN SHORTED TO GND - check the MOSFET wiring (gate must go to G, not S)");
    return false;
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  return true;
}

void stripPowerOn() {
  bool on1 = gateOn(STRIP_PWR_PIN, "STRIP 1");
  bool on2 = gateOn(STRIP2_PWR_PIN, "STRIP 2");
  delay(5);                    // let the strips' supply settle before data
  if (on1) { strip.begin();  strip.clear();  strip.show(); }    // data pin back to an output
  if (on2) { strip2.begin(); strip2.clear(); strip2.show(); }
}

void enterSleep() {
  if (asleep) return;
  asleep = true;
  imuConfigure(ODR_833HZ, ODR_OFF, false);   // gyro off, accel keeps feeding the tap detector
  fifoStart();
  resetTap();
  PDM.end();
  smoothedLevel = 0.0f;
  smoothedSpread = 0.0f;
  smoothedVolume = 0.0f;
  strip.clear();
  strip.show();
  strip2.clear();
  strip2.show();
  stripPowerOff();
  blinkStatus(LEDB);
  Serial.println("sleep");
}

void wakeUp() {
  if (!asleep) return;
  asleep = false;
  stripPowerOn();
  imuConfigure(ODR_833HZ, ODR_416HZ, false);
  fifoStart();
  resetTap();
  // Reset the idle clock here, not in the caller - otherwise waking after
  // ten minutes on the table would fall straight back asleep next iteration.
  lastActivityMs = millis();
  startMic();
  wakeAnimAt = millis();
  blinkStatus(LEDG);
  Serial.println("wake");
}

// Awake: any rotation above WAKE_GYRO_DPS counts as motion. Decided on the
// raw reading, not a smoothed one - smoothing would delay catching motion.
void checkIdle() {
  if (!imuOk) return;
  if (millis() - lastGyroCheck < GYRO_CHECK_MS) return;
  lastGyroCheck = millis();
  if ((int32_t)(millis() - imuSettleUntil) < 0) return;

  float gx = fabsf(imu.readFloatGyroX());
  float gy = fabsf(imu.readFloatGyroY());
  float gz = fabsf(imu.readFloatGyroZ());
  if (fmaxf(gx, fmaxf(gy, gz)) > WAKE_GYRO_DPS) {
    lastActivityMs = millis();
  } else if (millis() - lastActivityMs >= IDLE_SLEEP_MS) {
    enterSleep();
  }
}

void printTapDebug() {
  if (!tapDebugCount) return;
  if (!streaming) { Serial.print("TAPS "); Serial.println(tapDebugCount); }
  tapDebugCount = 0;
}

// Asleep: a double tap (or more) wakes it - without changing the mode.
void checkWake() {
  serviceAccel();
  printTapDebug();
  if (clickTriggered || tapCommand) {
    clickTriggered = false;
    tapCommand = 0;
    Serial.println("wake trigger: taps");
    wakeUp();
  }
}

// ---------------------------------------------------------------------------
// CHARGE METER (on a USB charger, strip 1 shows the battery level)
// ---------------------------------------------------------------------------
// USB power + no USB host = a charger (a computer sends a USB start-of-frame
// every 1ms even with no app open; a charger sends nothing). On a computer
// the mask runs normally, for testing.
//
// The charger (BQ25100) has no power path: whatever the board and strips draw
// comes out of its 100mA. So the meter isn't on all the time: it shows for
// CHARGE_SHOW_MS every CHARGE_PERIOD_MS, with both strips cut off in between,
// and everything else (mic, effects) is asleep while charging.
//
// The two 400mAh cells are in parallel = one 800mAh battery to the board.
// Level is estimated from the cell voltage; while charging the charger pushes
// the voltage up, so CHARGE_V_OFFSET is taken off first - the % is rough
// (optimistic). "Full" comes from the charger's own ~CHG pin, not the voltage.
#define PIN_CHG_STATUS     23      // P0.17, BQ25100 ~CHG: LOW = charging
#define PIN_HICHG          22      // P0.13: LOW = 100mA charge current (else 50mA)
#define PIN_VBAT_EN        32      // P0.14: keep LOW. HIGH while charging puts >3.3V on P0.31
#define VBAT_DIVIDER       (1510.0f / 510.0f)   // 1M over 510k
#define CHARGE_SHOW_MS     5000
#define CHARGE_PERIOD_MS   20000
#define CHARGE_V_OFFSET    0.08f   // V the charger adds on top of the resting voltage
#define CHARGE_LEVEL       0.35f   // perceived brightness of the meter

bool     chargeMode    = false;
uint32_t usbHostSeenMs = 0;
uint32_t usbLastFrame  = 0;
uint32_t chargerSinceMs = 0;      // 0 = no charger right now
uint32_t chargeShowUntil = 0, chargeNextShow = 0;
float    vbatSmoothed  = 0.0f;
bool     chargeStripsOn = false;

void initCharge() {
  pinMode(PIN_VBAT_EN, OUTPUT);
  digitalWrite(PIN_VBAT_EN, LOW);        // divider on, and safe while charging
  pinMode(PIN_HICHG, OUTPUT);
  digitalWrite(PIN_HICHG, LOW);          // 100mA: 800mAh of cells
  pinMode(PIN_CHG_STATUS, INPUT_PULLUP);
  analogReference(AR_INTERNAL2V4);       // 0.6V ref, gain 1/4: 2.4V full scale
  analogReadResolution(12);
}

float readVbat() {
  return analogRead(PIN_VBAT) * 2.4f / 4095.0f * VBAT_DIVIDER;
}

// Single-cell LiPo, resting voltage -> rough state of charge.
float vbatPercent(float v) {
  static const float V[] = {3.30f, 3.60f, 3.70f, 3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
  static const float P[] = {0.0f, 10.0f, 30.0f, 55.0f, 75.0f, 88.0f, 95.0f, 100.0f};
  if (v <= V[0]) return 0.0f;
  for (int k = 1; k < 8; k++) {
    if (v < V[k]) return P[k - 1] + (P[k] - P[k - 1]) * (v - V[k - 1]) / (V[k] - V[k - 1]);
  }
  return 100.0f;
}

bool usbPowered()  { return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0; }

void renderChargeMeter(bool charging) {
  float pct = charging ? vbatPercent(vbatSmoothed - CHARGE_V_OFFSET) : 100.0f;
  float fill = pct / 100.0f * NUM_LEDS1;               // strip 1 only, from its LED 0 up
  float breath = 0.55f + 0.45f * sinf(millis() * 0.004f);
  for (int i = 0; i < NUM_LEDS; i++) {
    pixDuty[i] = 0.0f;
    if (i >= NUM_LEDS1) continue;
    float frac = (float)i / (NUM_LEDS1 - 1);
    pixColor[i] = strip.ColorHSV((uint16_t)(frac * 21845.0f), 255, 255);   // red -> green along the bar
    float on = constrain(fill - i, 0.0f, 1.0f);       // partial top LED
    float level = CHARGE_LEVEL * on;
    if (charging && i == (int)fill) level = CHARGE_LEVEL * breath;          // the growing edge breathes
    pixDuty[i] = powf(level, LED_GAMMA);
  }
  showPixels();
}

// Returns true while the mask is in charge mode (loop() should do nothing else).
bool serviceCharge() {
  uint32_t now = millis();
  uint32_t frame = NRF_USBD->FRAMECNTR;
  if (frame != usbLastFrame) { usbLastFrame = frame; usbHostSeenMs = now; }
  bool host = (now - usbHostSeenMs) < 300;
  bool charger = usbPowered() && !host;

  if (!charger) {
    chargerSinceMs = 0;
    if (chargeMode) {                                  // unplugged, or plugged into a computer
      chargeMode = false;
      wakeUp();
    }
    return false;
  }
  if (!chargerSinceMs) chargerSinceMs = now;
  if (!chargeMode) {
    if (now - chargerSinceMs < 1500) return false;     // let a computer enumerate first
    chargeMode = true;
    enterSleep();                                      // mic/effects off, strips cut
    chargeStripsOn = false;
    vbatSmoothed = readVbat();
    chargeShowUntil = now + CHARGE_SHOW_MS;            // show right away
    chargeNextShow = now + CHARGE_PERIOD_MS;
  }

  vbatSmoothed += 0.05f * (readVbat() - vbatSmoothed);
  bool charging = digitalRead(PIN_CHG_STATUS) == LOW;
  if ((int32_t)(now - chargeNextShow) >= 0) {
    chargeShowUntil = now + CHARGE_SHOW_MS;
    chargeNextShow = now + CHARGE_PERIOD_MS;
  }
  bool show = (int32_t)(now - chargeShowUntil) < 0;
  if (show) {
    if (!chargeStripsOn) { stripPowerOn(); chargeStripsOn = true; }
    renderChargeMeter(charging);
    delay(30);
  } else {
    if (chargeStripsOn) { strip.clear(); strip.show(); strip2.clear(); strip2.show(); stripPowerOff(); chargeStripsOn = false; }
    delay(100);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  initCharge();

  strip.setBrightness(255);   // no global scaling - renderEffect() scales per pixel
  strip2.setBrightness(255);
  stripPowerOn();

  recomputeLayout();
  initPerlin();

  imu.settings.gyroSampleRate  = 416;
  imu.settings.accelSampleRate = 833;    // taps are a few ms long
  imu.settings.accelBandWidth  = 400;    // no extra low-pass smearing the taps
  imuOk = (imu.begin() == 0);
  if (!imuOk) Serial.println("IMU init failed - sleep mode disabled");
  if (imuOk) {
    Wire1.setClock(400000);  // the library leaves I2C at 100kHz - too slow to drain 833Hz xyz
    fifoStart();
    resetTap();
  }
  lastActivityMs = millis();   // idle clock starts at boot

  if (!startMic()) {
    Serial.println("PDM init failed - check board package (needs the mbed-enabled Seeed nRF52 core)");
    while (1) { delay(1000); }
  }
}

// 'R' starts a capture; once the buffer is full, dump it. The mask keeps
// running normally while it records.
void handleCapture() {
  static bool dumpPending = false;
  while (Serial.available()) {
    int cmd = Serial.read();
    if (cmd == 'R' && !asleep && !capturing && !dumpPending && !streaming) {
      captureCount = 0;
      capturing = true;
      dumpPending = true;
      Serial.println("CAPTURE_START");
    } else if (cmd == 'S' && !capturing && !dumpPending) {
      if (asleep) wakeUp();   // a session started on a sleeping board used to record nothing
      noInterrupts();
      streamTail = micSampleCount;
      streaming = true;
      interrupts();
      streamStartMs = millis();
    } else if (cmd == 'X') {
      streaming = false;
    } else if (cmd == 'P' && !streaming) {
      // debug: is the strip-power gate pin shorted? Pull it up, then down,
      // and read what it actually sits at, then drive it HIGH again.
      const int pins[2] = {STRIP_PWR_PIN, STRIP2_PWR_PIN};
      for (int s = 0; s < 2; s++) {
        int pin = pins[s];
        pinMode(pin, INPUT_PULLUP);   delay(5); int up = digitalRead(pin);
        pinMode(pin, INPUT_PULLDOWN); delay(5); int dn = digitalRead(pin);
        if (up) { pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); }
        else    { pinMode(pin, INPUT); }     // shorted: don't drive into it
        Serial.print("strip "); Serial.print(s + 1);
        Serial.print(" gate pin with pull-up: "); Serial.print(up);
        Serial.print("  with pull-down: "); Serial.println(dn);
      }
    } else if (cmd == 'K') {          // "K2,30.5" -> strip 2's center at LED 30.5
      Serial.setTimeout(100);
      int st = Serial.parseInt();
      float c = Serial.parseFloat();
      if (st == 1) strip1Center = c;
      if (st == 2) strip2Center = c;
      recomputeLayout();
      lastActivityMs = millis();
      Serial.print("CENTERS "); Serial.print(strip1Center, 1); Serial.print(" "); Serial.println(strip2Center, 1);
    } else if (cmd == 'O') {          // "O3" -> jump straight to mode 3 (0-based)
      Serial.setTimeout(100);
      int m = Serial.parseInt();
      if (m >= 0 && m < MODE_COUNT) mode = m;
      lastActivityMs = millis();
      Serial.print("MODE "); Serial.println(MODE_NAMES[mode]);
    } else if (cmd == 'Y') {          // replay the wake-up pulse (awake)
      if (!asleep) wakeAnimAt = millis();
    } else if (cmd == 'Z') {          // sleep now (testing the tap wake)
      if (!asleep) enterSleep();
    } else if (cmd == 'V') {          // sound floor on/off, same as 5 taps
      tapCommand = 5;
    } else if (cmd == 'W') {
      convDemo = !convDemo;
      lastActivityMs = millis();
      Serial.print("CONV DEMO "); Serial.println(convDemo ? "on" : "off");
    } else if (cmd == 'C') {
      if (asleep) wakeUp();
      layoutTestUntil = millis() + 15000;
      lastActivityMs = millis();
      Serial.println("LAYOUT TEST 15s");
    } else if (cmd == 'L') {
      if (asleep) wakeUp();
      Serial.setTimeout(100);
      int st = Serial.parseInt();   // "L2,57" -> strip 2, LED 57
      int ix = Serial.parseInt();
      findStrip = (st == 1 || st == 2) ? st : 0;
      findIndex = ix;
      lastActivityMs = millis();
      Serial.print("FIND strip "); Serial.print(findStrip); Serial.print(" led "); Serial.println(findIndex);
    } else if (cmd == 'B') {
      fifoBurst = !fifoBurst;
    } else if (cmd == 'M') {
      fifoMargin = !fifoMargin;
    } else if (cmd == 'D' && !streaming) {   // debug: dump IMU / FIFO registers
      const uint8_t regs[] = {0x0F, 0x10, 0x13, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x3A, 0x3B, 0x3C, 0x3D};
      for (uint8_t r : regs) {
        uint8_t v = 0;
        imu.readRegister(&v, r);
        Serial.print("reg 0x"); Serial.print(r, HEX); Serial.print(" = 0x"); Serial.println(v, HEX);
      }
      Serial.print("fifo read errors: "); Serial.println(fifoErrors);
      Serial.print("usb power: "); Serial.print(usbPowered());
      Serial.print("  usb frame: "); Serial.print(NRF_USBD->FRAMECNTR);
      Serial.print("  vbat: "); Serial.print(readVbat(), 3);
      Serial.print("V  ~CHG: "); Serial.print(digitalRead(PIN_CHG_STATUS));
      Serial.print("  charge mode: "); Serial.println(chargeMode);
    }
  }
  if (streaming && millis() - streamStartMs > STREAM_MAX_MS) streaming = false;
  if (dumpPending && !capturing) {
    dumpPending = false;
    lastActivityMs = millis();   // the dump takes a moment - don't fall asleep over it
    Serial.print("CAPTURE_BEGIN ");
    Serial.println(CAPTURE_SAMPLES);
    Serial.write((const uint8_t*)captureBuf, sizeof(captureBuf));
    Serial.println();
    Serial.println("CAPTURE_END");
  }
}

void loop() {
  updateStatusLed();
  if (serviceCharge()) return;     // on a USB charger: battery meter only
  handleCapture();

  if (asleep) {
    checkWake();
    delay(SLEEP_SAMPLE_MS);   // mbed idles the CPU during delay()
    return;
  }

  checkIdle();
  if (asleep) return;   // just fell asleep this tick

#if USE_CLICK_TRIGGER
  processClickFrames();
#endif
  serviceAccel();                   // FIFO -> double-tap detector (+ stream packets)
  printTapDebug();
  if (findStrip) lastActivityMs = millis();   // don't fall asleep while mapping LEDs
  if (offFadeAt) {                  // 3 taps: fading out, taps ignored meanwhile
    tapCommand = 0;
    clickTriggered = false;
    if (millis() - offFadeAt >= OFF_FADE_MS) {
      offFadeAt = 0;
      Serial.println("off (3 taps)");
      enterSleep();
      return;
    }
  }
  if (tapCommand == 2) { tapCommand = 0; clickTriggered = true; }
  if (clickTriggered) {
    clickTriggered = false;
    mode = (mode + 1) % MODE_COUNT;
    raceLastMs = 0;                 // race/party resume from where they were, no jump
    partyLastMs = 0;
    convLastMs = 0;
    triggerFlashRGB = 0xFFFFFF;
    triggerFlashAt = millis();
    lastActivityMs = millis();
    if (streaming) {
      streamEvent('T', micSampleCount, nullptr, 0);
    } else {
      Serial.print("TRIGGER mode=");
      Serial.println(MODE_NAMES[mode]);
    }
  }

  if (tapCommand == 3) {
    tapCommand = 0;
    offFadeAt = millis();
    wakeAnimAt = 0;
    triggerFlashAt = 0;
  }

  if (tapCommand == 4) {
    tapCommand = 0;
    dimMode = !dimMode;
    lastActivityMs = millis();
    if (!streaming) {
      Serial.print("DIM ");
      Serial.println(dimMode ? "on" : "off");
    }
  }

  if (tapCommand >= 5) {
    tapCommand = 0;
    setSoundFloor(!soundFloor);
    triggerFlashRGB = soundFloor ? 0x00FF00 : 0xFF0000;
    triggerFlashAt = millis();
    lastActivityMs = millis();
    if (!streaming) {
      Serial.print("SOUND FLOOR ");
      Serial.println(soundFloor ? "on" : "off");
    }
  }

  if (streaming) {
    lastActivityMs = millis();      // a recording session must not fall asleep
    if (imuOk) {
      float v[4] = {imu.readFloatAccelX(), imu.readFloatAccelY(), imu.readFloatAccelZ(), 0.0f};
      v[3] = fmaxf(fabsf(imu.readFloatGyroX()), fmaxf(fabsf(imu.readFloatGyroY()), fabsf(imu.readFloatGyroZ())));
      streamEvent('I', micSampleCount, v, 4);
    }
  }
  streamDrain();

  float rms = readMicRMS();
  if (rms < 0.0f) return;   // no new samples yet this tick

  float targetLevel = rmsToLevel(rms);
  if (lastDb > ACTIVITY_DB) lastActivityMs = millis();   // voice counts as activity
  smoothedLevel = smoothLevel(targetLevel, smoothedLevel);
  smoothedSpread = smoothLevel(powf(lastNormalized, SPREAD_EXPONENT), smoothedSpread);
  smoothedVolume = smoothLevel(lastNormalized, smoothedVolume);

  // Pitch only while there's actual voice - on silence YIN would lock onto
  // noise and drag the hue around between words.
  lastPitchHz = (lastDb > NOISE_FLOOR_DB) ? detectPitch() : 0.0f;
  updateHue(lastPitchHz);

#if DEBUG_PLOT_AUDIO
  if (Serial && !streaming) {
    Serial.print(lastDb);
    Serial.print(",");
    Serial.print(smoothedLevel * 100.0f);
    Serial.print(",");
    Serial.println(lastPitchHz);
  }
#endif

  renderEffect();
}
