import socket
import time

s = socket.socket()
s.bind(('0.0.0.0', 9999))
s.listen(1)
print("Time server running on port 9999...")
print("Press Ctrl+C to stop.")

while True:
    conn, addr = s.accept()
    timestamp = str(int(time.time()))
    conn.send(timestamp.encode())
    conn.close()
    print(f"Sent {timestamp} to {addr}")
