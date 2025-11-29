from pymavlink import mavutil
import sys

# CONFIGURATION
# Set this to the JETSON'S IP address (e.g. 10.151.210.21)
JETSON_IP = input("Enter Jetson IP: ") 
CMD_PORT = 14551

print(f"Connecting to Jetson at {JETSON_IP}:{CMD_PORT}...")
master = mavutil.mavlink_connection(f'udpout:{JETSON_IP}:{CMD_PORT}', source_system=255)

def send_action(action_id):
    # Sends a COMMAND_LONG with ID 31010
    # param1 = action_id
    master.mav.command_long_send(
        1, 100, # Target System, Target Component
        31010,  # Command ID (MAV_CMD_USER_1)
        0,      # Confirmation
        action_id, 0, 0, 0, 0, 0, 0 # Params
    )

print("\n--- COMMANDER READY ---")
print("Controls:")
print(" W - Fly North")
print(" S - Fly South")
print(" D - Fly East")
print(" A - Fly West")
print(" SPACE - Stop / Hover")
print(" Q - Quit")

while True:
    key = input("Command > ").upper()
    
    if key == 'W':
        send_action(1)
        print(">> Sending NORTH")
    elif key == 'S':
        send_action(2)
        print(">> Sending SOUTH")
    elif key == 'D':
        send_action(3)
        print(">> Sending EAST")
    elif key == 'A':
        send_action(4)
        print(">> Sending WEST")
    elif key == ' ':
        send_action(0)
        print(">> Sending STOP")
    elif key == 'Q':
        break