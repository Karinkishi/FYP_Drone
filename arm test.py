import time
from pymavlink import mavutil

# Set the connection parameters (update with your specific connection settings)
connection_string = '/dev/ttyUSB0'
baud_rate = 57600

# Create the MAVLink connection
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

# Wait for the heartbeat message to ensure the connection is established
master.wait_heartbeat()

# Set the system ID and component ID (update with your specific IDs)
system_id = 1
component_id = 1

# Send the MAV_CMD_DO_SET_MODE command to set the flight mode
mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
custom_mode = 4  # GUIDED mode in ArduPilot
base_mode = 0
master.mav.command_long_send(system_id, component_id,
                             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                             0, base_mode, custom_mode, 0, 0, 0, 0, 0)

# Wait for a short moment before sending the arming command
time.sleep(1)

# Send the MAV_CMD_COMPONENT_ARM_DISARM command to arm the drone
arm_command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
arm_params = [1, 0, 0, 0, 0, 0, 0]  # Set the arm_disarm parameter to 1 (arm)
master.mav.command_long_send(system_id, component_id,
                             arm_command,
                             0, *arm_params)

# Monitor the response from the flight controller
while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.command == arm_command:
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Arming command successful!")
        else:
            print("Arming command failed!")
        break

# Close the MAVLink connection
master.close()
