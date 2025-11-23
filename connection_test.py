cat << 'EOF' > connection_test.py
from pymavlink import mavutil
import sys

# CONFIGURATION
connection_string = '/dev/ttyACM0'
baud_rate = 115200

print(f"Connecting to Pixhawk on {connection_string}...")

try:
    master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
except Exception as e:
    print(f"Error connecting: {e}")
    sys.exit(1)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat received! System ID: {master.target_system}, Component ID: {master.target_component}")

print("Listening for Attitude Data (Press Ctrl+C to stop)...")
try:
    while True:
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            roll = msg.roll * 57.2958
            pitch = msg.pitch * 57.2958
            yaw = msg.yaw * 57.2958
            sys.stdout.write(f"\rRoll: {roll:.2f}° | Pitch: {pitch:.2f}° | Yaw: {yaw:.2f}°   ")
            sys.stdout.flush()
except KeyboardInterrupt:
    print("\nExiting...")
    master.close()
EOF