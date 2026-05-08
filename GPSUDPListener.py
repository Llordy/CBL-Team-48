#!/usr/bin/env python3
import json
import socket
from datetime import datetime

PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', PORT))

print(f"Listening for GPS packets on UDP port {PORT}...")
print("-" * 50)

while True:
    data, addr = sock.recvfrom(1024)
    try:
        d = json.loads(data.decode())
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] from {addr[0]}")
        print(f"  lat:      {d['latitude']:.6f}")
        print(f"  lon:      {d['longitude']:.6f}")
        print(f"  alt:      {d.get('altitude', 0.0):.1f} m")
        print(f"  accuracy: {d.get('accuracy', 0.0):.1f} m")
        print()
    except Exception as e:
        print(f"Bad packet from {addr}: {e}")
        print(f"  raw: {data}")
        print()
