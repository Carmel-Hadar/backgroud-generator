"""
LED finder: lights one LED at a time on the mask so you can see which number
sits where. Uses the sketch's "L<strip>,<index>" serial command.

Usage:  python led_finder.py [PORT]      (default COM3)
"""
import sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"

HELP = """
  ENTER        next LED
  -            previous LED
  <number>     jump to that LED     (e.g. 57)
  +<n> / -<n>  jump forward/back n  (e.g. +10)
  s1 / s2      switch to strip 1 / strip 2
  q            quit (mask goes back to normal)
"""


def main():
    ser = serial.Serial(PORT, 115200, timeout=0.2)
    ser.dtr = True
    time.sleep(0.5)
    strip, idx = 1, 0

    def show():
        ser.reset_input_buffer()
        ser.write(f"L{strip},{idx}\n".encode())

    print("X Voice - LED finder")
    print(HELP)
    show()
    while True:
        cmd = input(f"strip {strip}, LED {idx} is lit > ").strip().lower()
        if cmd == "q":
            break
        elif cmd == "":
            idx += 1
        elif cmd == "-":
            idx = max(0, idx - 1)
        elif cmd in ("s1", "s2"):
            strip, idx = int(cmd[1]), 0
        elif cmd[0] in "+-" and cmd[1:].isdigit():
            idx = max(0, idx + int(cmd))
        elif cmd.isdigit():
            idx = int(cmd)
        else:
            print(HELP)
            continue
        show()
    ser.write(b"L0,0\n")
    ser.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        input("\nPress ENTER to close.")
