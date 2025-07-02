import socket

# ----- CONFIGURATION -----
UDP_IP = "0.0.0.0"      # Listen on all interfaces
UDP_PORT = 12345        # Must match the sender's port
# -------------------------

def main():
    # Create the UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"[INFO] Listening for UDP packets on {UDP_IP}:{UDP_PORT} ...")

    try:
        while True:
            data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
            decoded = data.decode('utf-8', errors='ignore').strip()
            print(f"[RECEIVED from {addr[0]}:{addr[1]}] {decoded}")
    except KeyboardInterrupt:
        print("\n[EXIT] Receiver stopped by user.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
