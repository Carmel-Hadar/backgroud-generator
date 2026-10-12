"""
Center tuner: nudge each strip's X-center live (half-LED steps) and watch the
effects line up. Uses the sketch's K (center), O (mode), W (comet demo) and
C (layout test) serial commands. Values are RAM-only on the board - tell
Claude the final numbers to bake them into the code.

Usage:  python center_tuner.py [PORT]      (default COM3)
"""
import sys, time, threading
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"

HELP = """
  1+ / 1-     move strip 1's center half an LED up / down
  2+ / 2-     move strip 2's center half an LED up / down
  c           X converging, comets flying without talking (best for tuning)
  x           X mode (talk to see it)
  t           layout test: red = center, blue = ends (15s)
  q           quit
"""


def main():
    ser = serial.Serial(PORT, 115200, timeout=0.2)
    ser.dtr = True
    time.sleep(0.5)
    c1, c2 = 29.0, 31.0
    demo = False

    # Keep draining what the board prints. With the port open but nobody
    # reading, the board's USB writes back up and its whole loop freezes.
    stop = threading.Event()

    def drain():
        while not stop.is_set():
            try:
                ser.read(ser.in_waiting or 1)
            except Exception:
                break

    threading.Thread(target=drain, daemon=True).start()

    def send(cmd):
        ser.write(cmd.encode())
        time.sleep(0.15)

    def set_centers():
        send(f"K1,{c1}\n")
        send(f"K2,{c2}\n")

    def set_demo(on):
        nonlocal demo
        if demo != on:
            send("W")
            demo = on

    print("X Voice - center tuner")
    print(HELP)
    set_centers()
    send("O3\n")
    set_demo(True)
    while True:
        cmd = input(f"strip 1 center {c1:.1f}, strip 2 center {c2:.1f} > ").strip().lower()
        if len(cmd) == 2 and cmd[0] in "+-" and cmd[1] in "12":
            cmd = cmd[1] + cmd[0]          # accept "+1" as well as "1+"
        if cmd == "q":
            break
        elif cmd in ("1+", "1-", "2+", "2-"):
            d = 0.5 if cmd[1] == "+" else -0.5
            if cmd[0] == "1":
                c1 += d
            else:
                c2 += d
            set_centers()
        elif cmd == "c":
            send("O3\n"); set_demo(True)
        elif cmd == "x":
            set_demo(False); send("O0\n")
        elif cmd == "t":
            send("C")
        else:
            print(HELP)
    set_demo(False)
    stop.set()
    time.sleep(0.3)
    ser.close()
    print(f"\nFinal: strip 1 center {c1:.1f}, strip 2 center {c2:.1f}  - tell Claude these numbers.")
    input("Press ENTER to close.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        input("\nPress ENTER to close.")
