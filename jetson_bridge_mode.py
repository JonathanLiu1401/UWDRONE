cat << 'EOF' > jetson_bridge_mode.py
import time
import sys
from pymavlink import mavutil

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
# UPDATE THIS IP if your field laptop IP changes!
GCS_IP = '10.151.210.66' 
GCS_PORT = 14550

print(f"Connecting to Pixhawk on {SERIAL_PORT}...")
try:
    vehicle = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE, source_system=1)
except Exception as e:
    print(f"Error connecting: {e}")
    sys.exit(1)

print(f"Opening UDP Bridge to {GCS_IP}:{GCS_PORT}...")
gcs = mavutil.mavlink_connection(f'udpout:{GCS_IP}:{GCS_PORT}', source_system=1)

vehicle.wait_heartbeat()
print(f"Connected! Mode: {vehicle.flightmode}")

# --- VARIABLES ---
last_velocity_time = 0
VELOCITY_INTERVAL = 0.1 

def send_velocity_command(vx, vy, vz):
    # Mask: Ignore Pos/Accel, Enable Velocity + Yaw
    type_mask = int(0b110111000111)
    
    vehicle.mav.set_position_target_local_ned_send(
        0, vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        type_mask,
        0, 0, 0,    # Pos
        vx, vy, vz, # Vel (X=North, Y=East, Z=Down)
        0, 0, 0,    # Accel
        0, 0        # Yaw
    )

print("Bridge Active. Waiting for OFFBOARD/GUIDED mode...")

try:
    while True:
        # 1. Downlink
        msg = vehicle.recv_match(blocking=False)
        if msg:
            gcs.write(msg.get_msgbuf())

        # 2. Uplink
        gcs_msg = gcs.recv_match(blocking=False)
        if gcs_msg:
            vehicle.write(gcs_msg.get_msgbuf())

        # 3. Autonomy
        current_time = time.time()
        if (current_time - last_velocity_time > VELOCITY_INTERVAL):
            
            mode = vehicle.flightmode
            if mode == 'GUIDED' or mode == 'OFFBOARD': 
                # ACTIVE: Fly Forward (North) at 0.5 m/s
                send_velocity_command(0.5, 0, 0)
            else:
                # PASSIVE: Send 0,0,0 to keep Offboard link valid
                send_velocity_command(0, 0, 0)
            
            last_velocity_time = current_time
            
        time.sleep(0.001)

except KeyboardInterrupt:
    vehicle.close()
    gcs.close()
EOF