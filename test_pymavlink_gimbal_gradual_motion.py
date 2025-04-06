import time
from pymavlink import mavutil

# start connection
conn = mavutil.mavlink_connection("/dev/ttyUSB0")

# wait for first heartbeat
print("Waiting for Heartbeat...")
conn.wait_heartbeat()
print(f"Heartbeat from system (system {conn.target_system} component {conn.target_component})")

rotation_range = 45 
deg = -rotation_range

while deg<=rotation_range:
    print(f"Controlling gimbal {deg} degree")
    conn.mav.command_long_send(conn.target_system, 
                            conn.target_component,
                            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                            0,
                        ###    the parameters
                            0,   # parameter 1: pitch
                          deg,   # parameter 2: roll
                            0,   # parameter 3: yaw
                            0,   # parameter 4: (unused)
                            0,   # parameter 5: (unused)
                            0,   # parameter 6: input mode (position or speed)
                            2    # parameter 7: Mount Mode - MAV_MOUNT_MODE_MAVLINK_TARGETTING
                            )

    # msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
    # print(msg)
    print(f"Maneuver done!\n")
    deg += 5
    time.sleep(1)

print(f"Return to default position.")
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