"""
Live serial plotter for the X Voice mask (replacement for Arduino IDE's Serial Plotter).
Reads "dB,level*100[,pitchHz]" lines from the XIAO and graphs them live.
Also logs everything to serial_log.csv (next to this script) for calibration.

Usage:  python serial_plotter.py [PORT] [--calibrate]      (default COM3)
  --calibrate  guided run: the window shows what to do (quiet / talk / low /
               high / shout) with a countdown, and each row in the log is
               tagged with its stage so the thresholds can be computed per stage.
"""
import sys, os, time, threading, collections, math
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

args = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = args[0] if args else "COM3"
CALIBRATE = "--calibrate" in sys.argv

# (stage tag, on-screen instruction, seconds)
CAL_STAGES = [
    ("ready",  "GET READY...",                        3),
    ("quiet",  "STAY QUIET",                          6),
    ("talk",   "TALK NORMALLY",                       8),
    ("low",    "TALK IN YOUR LOWEST VOICE",           7),
    ("high",   "TALK IN YOUR HIGHEST VOICE",          7),
    ("shout",  "SHOUT",                               5),
]
cal_start = None   # set when the first line arrives


def cal_stage():
    """Current (tag, instruction, seconds left), or None when done / not calibrating."""
    if not CALIBRATE or cal_start is None:
        return None
    t = time.time() - cal_start
    for tag, text, dur in CAL_STAGES:
        if t < dur:
            return tag, text, dur - t
        t -= dur
    return None
BAUD = 115200
WINDOW = 600              # samples on screen (~15s)
NOISE_FLOOR_DB = 30.0     # keep in sync with the sketch
LOUD_DB = 60.0
PITCH_LOW_HZ = 90.0
PITCH_HIGH_HZ = 160.0

nan = float("nan")
db_buf = collections.deque([0.0] * WINDOW, maxlen=WINDOW)
lvl_buf = collections.deque([0.0] * WINDOW, maxlen=WINDOW)
pitch_buf = collections.deque([nan] * WINDOW, maxlen=WINDOW)
log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "serial_log.csv")


def reader():
    global cal_start
    with open(log_path, "w") as log:
        log.write("t,db,level_x100,pitch_hz,stage\n")
        while True:
            try:
                with serial.Serial(PORT, BAUD, timeout=1) as ser:
                    ser.dtr = True
                    t0 = time.time()
                    while True:
                        line = ser.readline().decode(errors="ignore").strip()
                        parts = line.split(",")
                        if len(parts) not in (2, 3):
                            continue
                        try:
                            vals = [float(p) for p in parts]
                        except ValueError:
                            continue
                        db, lvl = vals[0], vals[1]
                        hz = vals[2] if len(vals) == 3 else 0.0
                        db_buf.append(db)
                        lvl_buf.append(lvl)
                        pitch_buf.append(hz if hz > 0 else nan)
                        st = cal_stage()
                        tag = st[0] if st else ""
                        log.write(f"{time.time() - t0:.3f},{db},{lvl},{hz},{tag}\n")
                        log.flush()
            except serial.SerialException:
                time.sleep(1)   # board unplugged/resetting - retry


threading.Thread(target=reader, daemon=True).start()

fig, (ax, axp) = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                              gridspec_kw={"height_ratios": [3, 2]})
fig.canvas.manager.set_window_title(f"X Voice - live mic ({PORT})")
x = range(WINDOW)

db_line, = ax.plot(x, db_buf, label="mic dB", color="tab:blue")
lvl_line, = ax.plot(x, lvl_buf, label="LED level x100 (perceived, max 50)", color="tab:orange", linewidth=2)
ax.axhline(NOISE_FLOOR_DB, color="gray", ls="--", lw=1, label=f"NOISE_FLOOR_DB={NOISE_FLOOR_DB:g}")
ax.axhline(LOUD_DB, color="red", ls="--", lw=1, label=f"LOUD_DB={LOUD_DB:g}")
ax.set_ylim(0, 85)
ax.legend(loc="upper left", fontsize=8)
ax.grid(alpha=0.3)
title = ax.set_title("")
cal_text = fig.text(0.5, 0.965, "", ha="center", va="top", fontsize=22, fontweight="bold",
                    color="darkred") if CALIBRATE else None
if CALIBRATE:
    fig.subplots_adjust(top=0.86)

pitch_line, = axp.plot(x, pitch_buf, ".", color="tab:purple", markersize=3, label="pitch Hz")
axp.axhline(PITCH_LOW_HZ, color="red", ls="--", lw=1, label=f"PITCH_LOW_HZ={PITCH_LOW_HZ:g}")
axp.axhline(PITCH_HIGH_HZ, color="blue", ls="--", lw=1, label=f"PITCH_HIGH_HZ={PITCH_HIGH_HZ:g}")
axp.set_yscale("log")
axp.set_ylim(60, 520)
axp.set_yticks([70, 100, 150, 200, 300, 400, 500])
axp.get_yaxis().set_major_formatter(plt.ScalarFormatter())
axp.set_xlim(0, WINDOW - 1)
axp.set_xlabel("samples (~15s window)")
axp.legend(loc="upper left", fontsize=8)
axp.grid(alpha=0.3)


def update(_):
    db_line.set_ydata(db_buf)
    lvl_line.set_ydata(lvl_buf)
    pitch_line.set_ydata(pitch_buf)
    voiced = sorted(p for p in pitch_buf if not math.isnan(p))
    if voiced:
        n = len(voiced)
        ptxt = f"pitch p10/p50/p90: {voiced[n // 10]:.0f}/{voiced[n // 2]:.0f}/{voiced[int(n * .9)]:.0f} Hz"
    else:
        ptxt = "pitch: -"
    title.set_text(f"dB now: {db_buf[-1]:.1f}  |  peak: {max(db_buf):.1f}  |  level: {lvl_buf[-1]:.0f}  |  {ptxt}")
    if cal_text is not None:
        st = cal_stage()
        if cal_start is None:
            got_data = any(v != 0.0 for v in db_buf)
            cal_text.set_text("click here and press SPACE to start" if got_data else "waiting for the board...")
        elif st:
            cal_text.set_text(f"{st[1]}   ({st[2]:.0f})")
        else:
            cal_text.set_text("DONE - tell Claude")
            cal_text.set_color("darkgreen")
    return db_line, lvl_line, pitch_line, title


def on_key(event):
    global cal_start
    if CALIBRATE and event.key == " " and cal_start is None:
        cal_start = time.time()


fig.canvas.mpl_connect("key_press_event", on_key)

anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()
