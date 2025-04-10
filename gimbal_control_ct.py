from controller import PID
from pymavlink import mavutil
import time

def send_command_to_gimbal(conn, yaw, pitch):
    conn.mav.command_long_send(conn.target_system, 
                            conn.target_component,
                            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                            0,
                        ###    the parameters
                        pitch,   # parameter 1: pitch
                            0,   # parameter 2: roll
                          yaw,   # parameter 3: yaw
                            0,   # parameter 4: (unused)
                            0,   # parameter 5: (unused)
                            1,   # parameter 6: input mode (position or speed)
                            2    # parameter 7: Mount Mode - MAV_MOUNT_MODE_MAVLINK_TARGETTING
                            )
    time.sleep(0.01)

def gimbal_control(connection, position_queue):
    kp_x = 0.00035
    ki_x = 0.002
    kd_x = 0.0

    kp_y = 0.0001
    ki_y = 0.001
    kd_y = 0.0
    
    pid_x = PID(kp_x, ki_x, kd_x)
    pid_y = PID(kp_y, ki_y, kd_y)

    setpoint_yaw = 960
    setpoint_pitch = 540

    pid_x.SetPoint = setpoint_yaw
    pid_y.SetPoint = setpoint_pitch

    gimbal_x_pos = 0
    gimbal_y_pos = 0

    send_command_to_gimbal(connection, gimbal_x_pos, gimbal_y_pos)

    while True:
        position_x, position_y = position_queue.get()

        pid_x.update(position_x)
        pid_y.update(position_y)

        pid_control_x = pid_x.output
        pid_control_y = pid_y.output

        if position_x != 0:
            gimbal_x_pos = gimbal_x_pos + pid_control_x
            gimbal_y_pos = gimbal_y_pos + pid_control_y
            send_command_to_gimbal(connection, gimbal_x_pos, gimbal_y_pos)

        




