"""
Records voice-trigger samples from the XIAO's own mic (via the sketch's
RAW AUDIO CAPTURE mode) and saves each take as a 16kHz WAV.

Usage:  python capture_trigger.py [PORT] [TAKES] [LABEL]      (default COM3, 6, take)
Output: trigger_samples/<LABEL>_1.wav ... (next to this script)
"""
import sys, os, time, wave
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
TAKES = int(sys.argv[2]) if len(sys.argv) > 2 else 6
LABEL = sys.argv[3] if len(sys.argv) > 3 else "take"
RATE = 16000
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trigger_samples")
os.makedirs(out_dir, exist_ok=True)


def capture(ser):
    ser.reset_input_buffer()
    ser.write(b"R")
    t0 = time.time()
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "CAPTURE_START":
            print("   >>> SAY IT NOW <<<")
        if line.startswith("CAPTURE_BEGIN"):
            n = int(line.split()[1])
            data = ser.read(n * 2)
            if len(data) != n * 2:
                raise RuntimeError(f"short read: {len(data)} of {n * 2} bytes")
            return data
        if time.time() - t0 > 10:
            raise RuntimeError("no capture from the board - is it asleep? shake it and retry")


def peak_db(data):
    import array, math
    a = array.array("h", data)
    m = max(abs(v) for v in a) or 1
    return 20 * math.log10(m / 32768)


def main():
    print(f"X Voice - trigger recorder: '{LABEL}' ({TAKES} takes, 2.5s each)\n")
    print("Each take: press ENTER, wait for 3-2-1, then say the trigger right away.\n")
    ser = serial.Serial(PORT, 115200, timeout=1)
    ser.dtr = True
    time.sleep(1)
    take = 1
    while take <= TAKES:
        input(f"Take {take}/{TAKES} - press ENTER when ready...")
        for c in (3, 2, 1):
            print(f"   {c}...")
            time.sleep(0.7)
        try:
            data = capture(ser)
        except RuntimeError as e:
            print(f"   ERROR: {e}")
            continue
        path = os.path.join(out_dir, f"{LABEL}_{take}.wav")
        with wave.open(path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(RATE)
            w.writeframes(data)
        print(f"   saved {os.path.basename(path)} (peak {peak_db(data):.1f} dBFS)")
        again = input("   ENTER = keep, r + ENTER = redo this take: ").strip().lower()
        if again != "r":
            take += 1
    ser.close()
    print("\nDONE - tell Claude.")
    input("Press ENTER to close.")


if __name__ == "__main__":
    main()
