#!/usr/bin/env python3
import socket
import sys
import select
import termios
import tty
import atexit

# UDP listen & send configuration
LOCAL_IP        = "192.168.1.68"       # listen on all interfaces
LOCAL_PORT      = 9001            # where we receive robot telemetry
ROBOT_IP        = "192.168.1.84"  # robot's UDP listener
ROBOT_PORT      = 9000            # robot listens for TELEOP

# Set up sockets
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LOCAL_IP, LOCAL_PORT))
print(f"Listening for UDP on {LOCAL_IP}:{LOCAL_PORT}…")

# Save terminal settings so we can restore later
orig_settings = termios.tcgetattr(sys.stdin)
def restore_terminal():
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
atexit.register(restore_terminal)

# Put stdin into raw mode for single‐char reads
tty.setcbreak(sys.stdin.fileno())

def send_teleop(f, b, l, r):
    msg = f"TELEOP,{int(f)},{int(b)},{int(l)},{int(r)}"
    sock.sendto(msg.encode('utf-8'), (ROBOT_IP, ROBOT_PORT))
    print(f"[SEND] {msg}")

print("Press W/A/S/D for forward/left/back/right, Q to quit.")

try:
    while True:
        # Use select to wait for either UDP or keyboard (timseout 0.1s)
        rlist, _, _ = select.select([sock, sys.stdin], [], [], 0.1)

        # Handle UDP telemetry
        if sock in rlist:
            data, addr = sock.recvfrom(4096)
            text = data.decode('utf-8', errors='replace').strip()
            print(f"[{addr[0]}:{addr[1]}] {text}")

        # Handle keypress
        if sys.stdin in rlist:
            ch = sys.stdin.read(1).lower()
            if ch == 'q':
                print("Quitting.")
                break
            # Map keys to teleop bit flags
            f = b = l = r = 0
            if ch == 'w':
                f = 1
            elif ch == 's':
                b = 1
            elif ch == 'a':
                l = 1
            elif ch == 'd':
                r = 1
            else:
                continue
            send_teleop(f, b, l, r)

except KeyboardInterrupt:
    print("\nInterrupted, exiting.")
finally:
    restore_terminal()
    sock.close()
