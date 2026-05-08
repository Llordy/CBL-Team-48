#!/usr/bin/env python3
import json
import socket
import time
import math
import argparse

parser = argparse.ArgumentParser(description='Dummy GPS sender')
parser.add_argument('--host', default='127.0.0.1', help='Laptop IP (default: localhost)')
parser.add_argument('--port', type=int, default=5005)
parser.add_argument('--rate', type=float, default=1.0, help='Packets per second')
args = parser.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Starting position — Eindhoven city center
LAT  =  51.4416
LON  =   5.4697
ALT  =  17.0
RADIUS = 0.001   # degrees, roughly 100m circle

print(f"Sending dummy GPS to {args.host}:{args.port} at {args.rate} Hz")
print("Ctrl+C to stop\n")

t = 0.0
try:
    while True:
        # Walk in a small circle
        lat = LAT + RADIUS * math.sin(t)
        lon = LON + RADIUS * math.cos(t)
        alt = ALT + 2.0 * math.sin(t * 0.5)   # slight altitude variation

        payload = json.dumps({
            "latitude":  lat,
            "longitude": lon,
            "altitude":  alt,
            "accuracy":  4.0,
        }).encode()

        sock.sendto(payload, (args.host, args.port))
        print(f"  sent  lat={lat:.6f}  lon={lon:.6f}  alt={alt:.1f}m")

        t += 0.1
        time.sleep(1.0 / args.rate)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    sock.close()
