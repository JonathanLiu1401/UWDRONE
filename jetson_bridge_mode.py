cat << 'EOF' > jetson_bridge_mode.py
import time
import sys
from pymavlink import mavutil

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
GCS_IP = '10.151.210.66'  # <--- UPDATE THIS TO YOUR LAPTOP IP
GCS_PORT = 14550          # Port for QGroundControl
CMD_PORT = 14551          # Port for Custom Commands

print(f"Connecting to Pixhawk on {SERIAL_PORT}...")
try:
    vehicle = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE, source_system=1)
except Exception as e:
    print(f"Error connecting: {e}")
    sys.exit(1)

print(f"Opening UDP Bridge to {GCS_IP}:{GCS_PORT}...")
gcs = mavutil.mavlink_connection(f'udpout:{GCS_IP}:{GCS_PORT}', source_system=1)

print(f"Opening Command Listener on Port {CMD_PORT}...")
# 'udpin' means we LISTEN for incoming packets on this port
cmd_listener = mavutil.mavlink_connection(f'udpin:0.0.0.0:{CMD_PORT}', source_system=1)

vehicle.wait_heartbeat()
print(f"Connected! Mode: {vehicle.flightmode}")

# --- STATE VARIABLES ---
# Default velocity: 0,0,0 (Hover)
target_vx = 0
target_vy = 0
target_vz = 0
last_velocity_time = 0
VELOCITY_INTERVAL = 0.1 

def send_velocity_command(vx, vy, vz):
    type_mask = int(0b110111000111)
    vehicle.mav.set_position_target_local_ned_send(
        0, vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        type_mask, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
    )

print("Bridge Active. Listening for QGC (14550) and Custom Commands (14551)...")

try:
    while True:
        # 1. BRIDGE: Drone -> Laptop (Telemetry)
        msg = vehicle.recv_match(blocking=False)
        if msg:
            gcs.write(msg.get_msgbuf())

        # 2. BRIDGE: Laptop -> Drone (QGC Commands like Arm/Disarm)
        gcs_msg = gcs.recv_match(blocking=False)
        if gcs_msg:
            vehicle.write(gcs_msg.get_msgbuf())

        # 3. CUSTOM COMMAND LISTENER (Port 14551)
        # We listen for COMMAND_LONG messages with ID 31010 (MAV_CMD_USER_1)
        cmd_msg = cmd_listener.recv_match(blocking=False)
        if cmd_msg:
            if cmd_msg.get_type() == 'COMMAND_LONG' and cmd_msg.command == 31010:
                # Parse the parameters sent from Laptop
                action_id = int(cmd_msg.param1)
                
                if action_id == 0:   # STOP / HOVER
                    target_vx, target_vy, target_vz = 0, 0, 0
                    print("CMD: STOP")
                elif action_id == 1: # NORTH
                    target_vx, target_vy, target_vz = 0.5, 0, 0
                    print("CMD: NORTH")
                elif action_id == 2: # SOUTH
                    target_vx, target_vy, target_vz = -0.5, 0, 0
                    print("CMD: SOUTH")
                elif action_id == 3: # EAST
                    target_vx, target_vy, target_vz = 0, 0.5, 0
                    print("CMD: EAST")
                elif action_id == 4: # WEST
                    target_vx, target_vy, target_vz = 0, -0.5, 0
                    print("CMD: WEST")

        # 4. EXECUTE AUTONOMY (10Hz Loop)
        current_time = time.time()
        if (current_time - last_velocity_time > VELOCITY_INTERVAL):
            mode = vehicle.flightmode
            if mode == 'GUIDED' or mode == 'OFFBOARD': 
                # Send whatever the current target velocity is
                send_velocity_command(target_vx, target_vy, target_vz)
            else:
                # Heartbeat for manual modes
                send_velocity_command(0, 0, 0)
            
            last_velocity_time = current_time
            
        time.sleep(0.001)

except KeyboardInterrupt:
    vehicle.close()
    gcs.close()
    cmd_listener.close()
EOF