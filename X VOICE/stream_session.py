"""
Records a live session from the mask (the sketch's LIVE STREAM mode): raw
audio from the board's mic, every click candidate with its score, every
trigger the board fired, IMU readings, and every 833Hz accelerometer sample
from the IMU FIFO - all on one timeline.

Usage:  python stream_session.py [SECONDS] [LABEL] [PORT]     (default 60, session, COM3)
        python stream_session.py --taps [PORT]                 guided tap-trigger session
Output: sessions/<LABEL>_<timestamp>.wav + .events.csv + .accel.csv  (next to this script)
"""
import sys, os, time, struct, wave, threading
import serial

GUIDED = "--taps" in sys.argv
args = [a for a in sys.argv[1:] if not a.startswith("--")]
if GUIDED:
    LABEL = "taps"
    PORT = args[0] if args else "COM3"
else:
    SECONDS = float(args[0]) if len(args) > 0 else 60
    LABEL = args[1] if len(args) > 1 else "session"
    PORT = args[2] if len(args) > 2 else "COM3"
RATE = 16000
ACCEL_RAW_TO_G = 0.061 * 8 / 1000
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sessions")
os.makedirs(out_dir, exist_ok=True)
stamp = time.strftime("%Y%m%d_%H%M%S")
base = os.path.join(out_dir, f"{LABEL}_{stamp}")

# Guided tap session: (stage tag, instruction, seconds)
TAP_SCRIPT = [("ready", "GET READY - put the mask on", 6)]
TAP_SCRIPT += [("tap", f"DOUBLE TAP #{k} NOW  (then stay still)", 4) for k in range(1, 11)]
TAP_SCRIPT += [("move", "NO TAPS - dance, nod and shake your head, walk around", 30),
               ("handle", "NO TAPS - take the mask off and on, set it down, pick it up", 30)]
TAP_SCRIPT += [("tapmove", f"KEEP DANCING  +  DOUBLE TAP #{k} NOW !!", 5) for k in range(1, 9)]
TAP_SCRIPT += [("end", "done", 1)]


def beep(tag):
    """Audible cue for every 'tap now' instruction - the screen is easy to miss while moving."""
    if tag in ("tap", "tapmove"):
        try:
            import winsound
            winsound.Beep(1200, 120)
        except Exception:
            pass

latest_idx = [0]   # newest audio sample index received, to timestamp stage changes


def read_packets(ser, stop, raw_file=None):
    buf = bytearray()
    audio = {}      # first sample index -> bytes
    events = []     # (type, sample_idx, values...)
    accel = []      # (mic idx at read, overrun, [(x, y, z) raw])
    while not stop.is_set():
        chunk = ser.read(ser.in_waiting or 1)
        if raw_file and chunk:
            raw_file.write(chunk)     # everything hits the disk as it arrives
        buf += chunk
        while True:
            k = buf.find(b"\xA5\x5A")
            if k < 0:
                del buf[:-1]
                break
            if len(buf) < k + 5:
                del buf[:k]
                break
            typ = chr(buf[k + 2])
            ln = buf[k + 3] | (buf[k + 4] << 8)
            if len(buf) < k + 5 + ln:
                del buf[:k]
                break
            p = bytes(buf[k + 5:k + 5 + ln])
            del buf[:k + 5 + ln]
            if typ == "A":
                idx = struct.unpack_from("<I", p)[0]
                audio[idx] = p[4:]
                latest_idx[0] = max(latest_idx[0], idx + (len(p) - 4) // 2)
            elif typ == "F":
                idx, over, n = struct.unpack_from("<IBB", p)
                xyz = struct.unpack_from("<%dh" % (3 * n), p, 6)
                accel.append((idx, over, [xyz[3 * j:3 * j + 3] for j in range(n)]))
            elif typ in "CTI":
                idx = struct.unpack_from("<I", p)[0]
                vals = struct.unpack_from("<%df" % ((ln - 4) // 4), p, 4)
                events.append((typ, idx) + vals)
                # no print() here: if the console freezes (a click in the window
                # pauses it), a print would block this thread, the board's USB
                # writes would back up, and the recording would stall
    return audio, events, accel


def main():
    total = sum(s[2] for s in TAP_SCRIPT) if GUIDED else SECONDS
    print(f"X Voice - live session '{LABEL}', {total:.0f}s\n")
    if GUIDED:
        print("Instructions will appear here. Each 'DOUBLE TAP' line (with a beep) = tap twice on the device, once.")
        print("Don't click inside this window while recording - it freezes it.\n")
    input("Press ENTER to start recording...")
    ser = serial.Serial(PORT, 115200, timeout=0.05)
    ser.dtr = True
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.write(b"S")
    stop = threading.Event()
    result = {}
    stages = []     # (mic sample index at the moment the instruction appeared, tag, text)
    raw_file = open(base + ".raw", "wb")

    def worker():
        try:
            result.update(zip(("audio", "events", "accel"), read_packets(ser, stop, raw_file)))
        except Exception:
            import traceback
            result["error"] = traceback.format_exc()

    th = threading.Thread(target=worker)
    th.start()
    t0 = time.time()
    warned = False
    if GUIDED:
        time.sleep(1.0)   # let the stream start before the first instruction
        for tag, text, dur in TAP_SCRIPT:
            stages.append((latest_idx[0], tag, text))
            if tag == "tapmove" and stages[-2][1] != "tapmove":
                print("\n" + "=" * 60 + "\n  NOW: KEEP DANCING, AND DOUBLE TAP ON EVERY BEEP\n" + "=" * 60)
            print(f"\n>>> {text}")
            beep(tag)
            end = time.time() + dur
            while time.time() < end:
                if not warned and os.path.getsize(base + ".raw") == 0:
                    print("   WARNING: no data from the board - is it connected? (Ctrl+C to abort)")
                    warned = True
                print(f"\r      {end - time.time():3.0f}s ", end="", flush=True)
                time.sleep(0.2)
        print()
    else:
        while time.time() - t0 < SECONDS:
            if not warned and time.time() - t0 > 3 and os.path.getsize(base + ".raw") == 0:
                print("\n   WARNING: no data from the board yet - is it connected? (Ctrl+C to abort)")
                warned = True
            left = SECONDS - (time.time() - t0)
            print(f"\r   recording... {left:4.0f}s left ", end="", flush=True)
            time.sleep(0.5)
        print()
    ser.write(b"X")
    time.sleep(0.3)
    stop.set()
    th.join()
    ser.close()
    raw_file.close()
    if "error" in result:
        print("ERROR while reading (raw data is still saved in the .raw file):")
        print(result["error"])
        input("Press ENTER to close.")
        return

    audio, events, accel = result["audio"], result["events"], result["accel"]
    if not audio:
        print("No audio received - is the board awake? Shake it and retry.")
        input("Press ENTER to close.")
        return
    first = min(audio)
    last = max(i + len(b) // 2 for i, b in audio.items())
    pcm = bytearray(2 * (last - first))       # gaps (if the board fell behind) stay silent
    gaps = 0
    expect = first
    for i in sorted(audio):
        if i != expect:
            gaps += 1
        pcm[2 * (i - first):2 * (i - first) + len(audio[i])] = audio[i]
        expect = i + len(audio[i]) // 2
    with wave.open(base + ".wav", "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(RATE); w.writeframes(pcm)
    with open(base + ".events.csv", "w") as f:
        f.write("type,t,v1,v2,v3,v4\n")
        for e in events:
            f.write(f"{e[0]},{(e[1] - first) / RATE:.4f}," + ",".join(f"{v:.4f}" for v in e[2:]) + "\n")
        for idx, tag, text in stages:
            f.write(f"S,{(idx - first) / RATE:.4f},{tag}\n")
    overruns = 0
    n_acc = 0
    with open(base + ".accel.csv", "w") as f:
        # k = running sample number at 833Hz; t_read = mic time when the batch was read
        f.write("k,t_read,overrun,ax,ay,az\n")
        for idx, over, samples in accel:
            overruns += over
            for x, y, z in samples:
                f.write(f"{n_acc},{(idx - first) / RATE:.4f},{over},"
                        f"{x * ACCEL_RAW_TO_G:.4f},{y * ACCEL_RAW_TO_G:.4f},{z * ACCEL_RAW_TO_G:.4f}\n")
                n_acc += 1
    trig = sum(1 for e in events if e[0] == "T")
    dur = (last - first) / RATE
    print(f"\nSaved {os.path.basename(base)}: {dur:.1f}s audio (gaps: {gaps}), "
          f"{n_acc} accel samples ({n_acc / max(dur, 1e-6):.0f}/s, FIFO overruns: {overruns})")
    print(f"Triggers fired during the session: {trig}")
    print("\nDONE - tell Claude.")
    input("Press ENTER to close.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        input("\nPress ENTER to close.")
