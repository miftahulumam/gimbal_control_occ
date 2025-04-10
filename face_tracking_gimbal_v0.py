
import pyrealsense2 as rs
from pymavlink import mavutil

import cv2
import numpy as np
import time
import datetime
import serial
import matplotlib.pyplot as plt
from matplotlib import animation

from controller import PID

cam_source = 0

prototxt_face = "./model/face_deploy.prototxt"
weight_face = "./model/face_model.caffemodel"
model_face = cv2.dnn.readNet(prototxt_face,weight_face)

model_face.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
model_face.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

frame_width = 552
frame_height = 552

W = None
H = None

min_confidence = 0.8
skip_frames = 1
totalFrames = 0
totalDown = 0

color = (255, 0, 0)
dim = (480,270)
rsz_dim = (frame_width, frame_height)

blue = (255,0,0)
pink = (127,0,255)
green = (0,255,0)

color_rect = green
color_text = green

prev_violate = 0
new_violate = 0
tot_violate = 0
tot_violate_all = 0

rsz_dim_face = (227, 227)
confi = 0

fps_timer_start = time.time()

mean = np.array([1.0, 1.0, 1.0]) * 127.5
scale = 1 / 255

# for control signal plotting
x_len = 300
xs = list(range(0, x_len))
y_range = [-30, 30]

ctrl_sig_x = [0] * x_len
ctrl_sig_y = [0] * x_len

ctrl_sig_x_deg = [0] * x_len
ctrl_sig_y_deg = [0] * x_len

setpoint_x = [0] * x_len
setpoint_y = [0] * x_len

feedback_x = [0] * x_len  # plot with the setpoint x
feedback_y = [0] * x_len  # plot with the setpoint y

# fig = plt.figure()
# ax = fig.add_subplot(1, 1, 1)
# ax.set_ylim(y_range) 
# ax.set_xlim(0, x_len)
# line_x, = ax.plot(xs,ctrl_sig_x)

plt.rcParams['figure.figsize'] = [15, 7]
fig, axs = plt.subplots(2,3)

fig.suptitle('Control System Interface')

axs[0,0].set_ylim(y_range)
axs[1,0].set_ylim(y_range)
axs[0,1].set_ylim([0, 180])
axs[1,1].set_ylim([0, 180])
axs[0,2].set_ylim([-10, 350])
axs[1,2].set_ylim([-10, 350])

axs[0,0].set_xlim(0, x_len)
axs[1,0].set_xlim(0, x_len)
axs[0,1].set_xlim(0, x_len)
axs[1,1].set_xlim(0, x_len)
axs[0,2].set_xlim(0, x_len)
axs[1,2].set_xlim(0, x_len)

axs[0,0].title.set_text('PID Yaw Control')
axs[1,0].title.set_text('PID Pitch Control')
axs[0,1].title.set_text('Yaw Servo Command')
axs[1,1].title.set_text('Pitch Servo Command')
axs[0,2].title.set_text('Yaw Setpoint vs Yaw Output')
axs[1,2].title.set_text('Pitch Setpoint vs Pitch Output')

line_ctrl_x, = axs[0,0].plot(xs,ctrl_sig_x)
line_ctrl_y, = axs[1,0].plot(xs,ctrl_sig_y)
line_ctrl_x_deg, = axs[0,1].plot(xs,ctrl_sig_x_deg)
line_ctrl_y_deg, = axs[1,1].plot(xs,ctrl_sig_y_deg)
line_fdbk_x, = axs[0,2].plot(xs,feedback_x)
line_fdbk_y, = axs[1,2].plot(xs,feedback_y)
line_setpt_x, = axs[0,2].plot(xs,setpoint_x)
line_setpt_y, = axs[1,2].plot(xs,setpoint_y)

plt.tight_layout()

# real sense camera
pipeline = rs.pipeline()
config = rs.config()

# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
# device = pipeline_profile.get_device()
# device_product_line = str(device.get_info(rs.camera_info.product_line))

# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) #for depth
config.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30) #for rgb
pipeline.start(config)

# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=.1)
# start connection
conn = mavutil.mavlink_connection("/dev/ttyUSB0")

# wait for first heartbeat
print("Waiting for Heartbeat...")
conn.wait_heartbeat()
print(f"Heartbeat from system (system {conn.target_system} component {conn.target_component})")

# def write_read(x): 
#     # x = str(x)
#     # print(x)
#     ser.write(bytes(x, 'utf-8')) 
#     time.sleep(0.05) 
#     data = ser.readline() 
#     print('from serial', data)
#     return data 

def send_command_to_gimbal(yaw, pitch):
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
                            0,   # parameter 6: input mode (position or speed)
                            2    # parameter 7: Mount Mode - MAV_MOUNT_MODE_MAVLINK_TARGETTING
                            )
    time.sleep(0.1)

def find_face(result):
    (startX_face, startY_face, endX_face, endY_face), center_coord = (0,0,0,0), (0,0)
    for i in range(0,result.shape[2]):
        confidence = result[0,0,i,2]

        if confidence > min_confidence:
            face_box = result[0,0,i,3:7] * np.array([W,H,W,H])
            (startX_face, startY_face, endX_face, endY_face) = face_box.astype("int")
        
            center_coord = (int((endX_face-startX_face)/2)+startX_face, int((endY_face-startY_face)/2)+startY_face)
    
    return (startX_face, startY_face, endX_face, endY_face), center_coord

def animate_control_x(i, line, ctrl_sig_x, ctrl_x):
    ctrl_x = round(ctrl_x, 2)

    ctrl_sig_x.append(ctrl_x)
    ctrl_sig_x = ctrl_sig_x[-x_len:]

    line.set_data(xs, ctrl_sig_x)

def animate_output(i, 
                   # lines
                   line_x, line_y, 
                   line_x_deg, line_y_deg, 
                   line_fdbk_x, line_fdbk_y,
                   line_stpt_x, line_stpt_y,
                   # lists
                   ctrl_sig_x, ctrl_sig_y, 
                   ctrl_sig_x_deg, ctrl_sig_y_deg,
                   feedback_x, feedback_y,
                   setpoint_x, setpoint_y,
                   # input
                   ctrl_x, ctrl_y, 
                   ctrl_x_deg, ctrl_y_deg,
                   fdbk_x, fdbk_y,
                   set_x, set_y
                   ):
    ctrl_x = round(ctrl_x, 2)
    ctrl_y = round(ctrl_y, 2)
    ctrl_x_deg = round(ctrl_x_deg, 2)
    ctrl_y_deg = round(ctrl_y_deg, 2)
    fdbk_x = round(fdbk_x, 2)
    fdbk_y = round(fdbk_y, 2)

    ctrl_sig_x.append(ctrl_x)
    ctrl_sig_y.append(ctrl_y)
    ctrl_sig_x_deg.append(ctrl_x_deg)
    ctrl_sig_y_deg.append(ctrl_y_deg)
    feedback_x.append(fdbk_x)
    feedback_y.append(fdbk_y)
    setpoint_x.append(set_x)
    setpoint_y.append(set_y)

    ctrl_sig_x = ctrl_sig_x[-x_len:]
    ctrl_sig_y = ctrl_sig_y[-x_len:]
    ctrl_sig_x_deg = ctrl_sig_x_deg[-x_len:]
    ctrl_sig_y_deg = ctrl_sig_y_deg[-x_len:]
    feedback_x = feedback_x[-x_len:]
    feedback_y = feedback_y[-x_len:]
    setpoint_x = setpoint_x[-x_len:]
    setpoint_y = setpoint_y[-x_len:]

    line_x.set_data(xs, ctrl_sig_x)
    line_y.set_data(xs, ctrl_sig_y)
    line_x_deg.set_data(xs, ctrl_sig_x_deg)
    line_y_deg.set_data(xs, ctrl_sig_y_deg)
    line_fdbk_x.set_data(xs, feedback_x)
    line_fdbk_y.set_data(xs, feedback_y)
    line_stpt_x.set_data(xs, setpoint_x)
    line_stpt_y.set_data(xs, setpoint_y)

            
if __name__ == '__main__':
    kp_x = 0.05
    ki_x = 0.3
    kd_x = 0.0

    kp_y = 0.03
    ki_y = 0.1
    kd_y = 0.0
    
    pid_x = PID(kp_x, ki_x, kd_x)
    pid_y = PID(kp_y, ki_y, kd_y)

    setpoint_yaw = 320
    setpoint_pitch = 240

    pid_x.SetPoint = setpoint_yaw
    pid_y.SetPoint = setpoint_pitch

    gimbal_x_pos = 0
    gimbal_y_pos = 0

    send_command_to_gimbal(gimbal_x_pos, gimbal_y_pos)
    
    try:
        while True:

            frames = pipeline.wait_for_frames()
            frame = frames.get_color_frame()
            if not frame:
                continue
            
            totalFrames += 1
            frame = np.asanyarray(frame.get_data())
            frame = cv2.resize(frame,rsz_dim)

            if W is None or H is None:
                (H, W) = frame.shape[:2]

            rects = []

            if totalFrames % skip_frames == 0:
                img_blob = cv2.dnn.blobFromImage(frame, 1.0, (300,300), (104.0, 177.0, 123.0))
                model_face.setInput(img_blob)
                result = model_face.forward()

                (startX_face, startY_face, endX_face, endY_face), center_coord = find_face(result)
                cv2.rectangle(frame, (startX_face, startY_face), (endX_face,endY_face), color_rect, 2)
                cv2.circle(frame, center_coord, 4, color_rect, 4)
                
                pid_x.update(center_coord[0])
                pid_y.update(center_coord[1])
                
                pid_control_x = pid_x.output
                pid_to_angle_x = np.interp(pid_control_x, [-65.2, 45.2], [0, 180])
                
                pid_control_y = pid_y.output
                pid_to_angle_y = np.interp(pid_control_y, [-48.9, 33.9], [170, 0])
                
                if center_coord[0] != 0:
                    gimbal_x_pos = gimbal_x_pos + pid_control_x
                    gimbal_y_pos = gimbal_y_pos + pid_control_y
                    send_command_to_gimbal(gimbal_x_pos, gimbal_y_pos)
                    # write_read('x: ' + str(int(pid_to_angle_x)) + 'y: ' + str(int(pid_to_angle_y)))
                    # write_read('x: ' + str(int(pid_to_angle_x)))
                    # write_read('y: ' + str(int(pid_to_angle_y)))
                    
                    print('PID output', pid_control_x, pid_to_angle_x, center_coord)
                    print('PID y output', pid_control_y, pid_to_angle_y)

            time_elapsed = time.time() - fps_timer_start
            fps = totalFrames / time_elapsed
            text_fps = 'FPS: {:.2f}'.format(fps)
            cv2.putText(frame, text_fps, (10, frame.shape[0] - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 255), 3)
            
            # control signal plot
            # anim = animation.FuncAnimation(fig, animate_control_x, fargs=(line_x, ctrl_sig_x, pid_control_x), interval=50) #, blit=True, cache_frame_data=False)
            anim = animation.FuncAnimation(fig, animate_output, 
                                           fargs=(line_ctrl_x, line_ctrl_y,
                                                  line_ctrl_x_deg, line_ctrl_y_deg,
                                                  line_fdbk_x, line_fdbk_y,
                                                  line_setpt_x, line_setpt_y,
                                                  ctrl_sig_x, ctrl_sig_y, 
                                                  ctrl_sig_x_deg, ctrl_sig_y_deg,
                                                  feedback_x, feedback_y,
                                                  setpoint_x, setpoint_y,
                                                  pid_control_x, pid_control_y,
                                                  pid_to_angle_x, pid_to_angle_y,
                                                  center_coord[0], center_coord[1],
                                                  setpoint_yaw, setpoint_pitch), 
                                           interval=1) #, cache_frame_data=False)

            fig.canvas.draw()
            img_plot = np.array(fig.canvas.renderer.buffer_rgba())

            cv2.namedWindow('Control System Interface', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Control System Interface', cv2.cvtColor(img_plot, cv2.COLOR_RGBA2BGR))
            
            # display object detection
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense',
                        frame)

            if cv2.waitKey(1) == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
