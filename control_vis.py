import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import cv2

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
axs[0,2].set_ylim([-10, 1920])
axs[1,2].set_ylim([-10, 1080])

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

def run_anim(control_vis_queue):
    # control signal plot

    while True:
        control_vis = control_vis_queue.get()
        
        pid_control_x = control_vis['control_signal_x']
        pid_control_y = control_vis['control_signal_y']
        position_x = control_vis['feedback_x']
        position_y = control_vis['feedback_y']
        setpoint_yaw = control_vis['setpoint_x']
        setpoint_pitch = control_vis['setpoint_y']


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
                                                pid_control_x, pid_control_y,
                                                position_x, position_y,
                                                setpoint_yaw, setpoint_pitch), 
                                            interval=1) #, cache_frame_data=False)

        fig.canvas.draw()
        img_plot = np.array(fig.canvas.renderer.buffer_rgba())

        cv2.namedWindow('Control System Interface', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Control System Interface', cv2.cvtColor(img_plot, cv2.COLOR_RGBA2BGR))

        if cv2.waitKey(1) == ord('q'):
            break