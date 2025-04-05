import time
from pymavlink import mavutil

# degree command
deg = {'yaw': 90, 'pitch': 0, 'roll': 0}

# start connection
conn = mavutil.mavlink_connection("/dev/ttyUSB0")

# wait for first heartbeat
print("Waiting for Heartbeat...")
conn.wait_heartbeat()
print(f"Heartbeat from system (system {conn.target_system} component {conn.target_component})")

# gimbal control 
print(f"Controlling gimbal: \nyaw = {deg['yaw']} \npitch = {deg['pitch']} \nroll = {deg['roll']}")
conn.mav.command_long_send(conn.target_system, 
                           conn.target_component,
                           mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                           0,
                    ###    the parameters
                deg['pitch'],   # parameter 1: pitch
                 deg['roll'],   # parameter 2: roll
                  deg['yaw'],   # parameter 3: yaw
                           0,   # parameter 4: (unused)
                           0,   # parameter 5: (unused)
                           0,   # parameter 6: input mode (position or speed)
                           2    # parameter 7: Mount Mode - MAV_MOUNT_MODE_MAVLINK_TARGETTING
                           )

# msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
# print(msg)
print(f"Maneuver done!\n")

time.sleep(2)

print(f"Controlling gimbal: \nyaw = {-deg['yaw']} \npitch = {-deg['pitch']} \nroll = {-deg['roll']}")
conn.mav.command_long_send(conn.target_system, 
                           conn.target_component,
                           mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                           0,
                    ###    the parameters
               -deg['pitch'],   # parameter 1: pitch
                -deg['roll'],   # parameter 2: roll
                 -deg['yaw'],   # parameter 3: yaw
                           0,   # parameter 4: (unused)
                           0,   # parameter 5: (unused)
                           0,   # parameter 6: input mode (position or speed)
                           2    # parameter 7: Mount Mode - MAV_MOUNT_MODE_MAVLINK_TARGETTING
                           )

# msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
# print(msg)
print(f"Maneuver done!\n")

time.sleep(2)

print(f"Controlling gimbal: \nyaw = 0 \npitch = 0\nroll = 0")
conn.mav.command_long_send(conn.target_system, 
                           conn.target_component,
                           mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                           0,
                    ###    the parameters
                           0,   # parameter 1: pitch
                           0,   # parameter 2: roll
                           0,   # parameter 3: yaw
                           0,   # parameter 4: (unused)
                           0,   # parameter 5: (unused)
                           0,   # parameter 6: input mode (position or speed)
                           2    # parameter 7: Mount Mode - MAV_MOUNT_MODE_MAVLINK_TARGETTING
                           )

# msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
# print(msg)
print(f"Maneuver done!\n")