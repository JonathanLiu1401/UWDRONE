cat << 'EOF' > jetson_bridge_mode.py
import time
import sys
from pymavlink import mavutil

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# UPDATE THIS IP to match your Ground Station Laptop
GCS_IP = '192.168.88.2' 
GCS_PORT = 14550

# --- SETUP ---
print(f"Connecting to Pixhawk on {SERIAL_PORT}...")
try:
    # source_system=1 makes the drone look like "System 1" to QGC
    vehicle = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE, source_system=1)
except Exception as e:
    print(f"Error connecting to vehicle: {e}")
    sys.exit(1)

print(f"Opening UDP Bridge to {GCS_IP}:{GCS_PORT}...")
gcs = mavutil.mavlink_connection(f'udpout:{GCS_IP}:{GCS_PORT}', source_system=1)

print("Waiting for vehicle heartbeat...")
vehicle.wait_heartbeat()
print(f"Vehicle Connected! Mode: {vehicle.flightmode}")

# --- VARIABLES ---
last_velocity_time = 0
VELOCITY_INTERVAL = 0.1 # 10Hz

def send_velocity_command(vx, vy, vz):
    # Bitmask to ignore Position/Accel, Enable Velocity + Yaw
    type_mask = int(0b110111000111)
    
    vehicle.mav.set_position_target_local_ned_send(
        0, vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        type_mask,
        0, 0, 0,    # Pos
        vx, vy, vz, # Vel
        0, 0, 0,    # Accel
        0, 0        # Yaw
    )

print("Bridge Active. Waiting for GUIDED mode...")

try:
    while True:
        # 1. DRONE -> GCS (Downlink)
        msg = vehicle.recv_match(blocking=False)
        if msg:
            # FIX: Send the raw bytes buffer, not the object
            gcs.write(msg.get_msgbuf())

        # 2. GCS -> DRONE (Uplink)
        gcs_msg = gcs.recv_match(blocking=False)
        if gcs_msg:
            # FIX: Send the raw bytes buffer here too
            vehicle.write(gcs_msg.get_msgbuf())

        # 3. AUTONOMY LOGIC
        current_time = time.time()
        if (current_time - last_velocity_time > VELOCITY_INTERVAL):
            
            # Check Flight Mode
            # Note: We rely on pymavlink's internal state tracking
            if vehicle.flightmode == 'GUIDED': 
                send_velocity_command(0.5, 0, 0)
                # print("Autonomy Active: Flying North", end='\r')
            
            last_velocity_time = current_time
            
        time.sleep(0.001)

except KeyboardInterrupt:
    print("\nBridge Stopped.")
    vehicle.close()
    gcs.close()
EOF