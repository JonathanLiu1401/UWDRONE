cat << 'EOF' > velocity_test.py
from pymavlink import mavutil
import time
import sys

# --- CONFIGURATION ---
# Using the USB connection established earlier
connection_string = '/dev/ttyACM0'
baud_rate = 115200

print(f"Connecting to Pixhawk on {connection_string}...")
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}, Component ID: {master.target_component}")

def send_velocity_command(vx, vy, vz):
    """
    Sends a velocity command to the drone in the NED (North-East-Down) frame.
    vx: Velocity North (m/s)
    vy: Velocity East (m/s)
    vz: Velocity Down (m/s) -- Negative is UP
    """
    
    # BITMASK: 0 = Enable, 1 = Ignore
    # We want to IGNORE Position (bits 0-2) and Acceleration (bits 6-8)
    # We want to USE Velocity (bits 3-5) and Yaw (bit 10)
    type_mask = int(0b110111000111)

    master.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (0=use system time)
        master.target_system,       # Target System ID
        master.target_component,    # Target Component ID
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Coordinate Frame
        type_mask,                  # Bitmask
        0, 0, 0,                    # Positions (Ignored)
        vx, vy, vz,                 # Velocities (Used)
        0, 0, 0,                    # Accelerations (Ignored)
        0, 0                        # Yaw, Yaw Rate
    )

print("Sending VELOCITY command: Move North at 0.5 m/s")
print("NOTE: Drone will NOT move unless in GUIDED/OFFBOARD mode and ARMED.")

try:
    while True:
        # Send the command at 10Hz (Standard for control loops)
        send_velocity_command(0.5, 0, 0)
        
        # Print confirmation
        sys.stdout.write("\rSending: Vx=0.5 m/s (North)...")
        sys.stdout.flush()
        
        time.sleep(0.1) # 10Hz

except KeyboardInterrupt:
    print("\nStopped.")
    master.close()
EOF