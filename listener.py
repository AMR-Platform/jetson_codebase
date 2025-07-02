import socket

# Must match the IP and port used by Jetson's UDP sender
UDP_IP = '0.0.0.0'        # Listen on all interfaces
UDP_PORT = 12345          # Must match what Jetson is sending to

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"[LISTENING] on port {UDP_PORT}...")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"[RECEIVED from {addr}] {data.decode().strip()}")
except KeyboardInterrupt:
    print("\n[EXIT] Interrupted.")
finally:
    sock.close()
