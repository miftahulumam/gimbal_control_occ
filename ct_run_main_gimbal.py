import os
import datetime
from multiprocessing import Process, Queue

import cv2
import numpy as np
from pymavlink import mavutil

from ct_cam_init import grab_image
from obj_det_gimbal_tracking import obj_detector
from gimbal_control_ct import gimbal_control as gimbal_control_v0
from gimbal_control_ct_visual import gimbal_control as gimbal_control_v1
from visualization import image_show
from control_vis import run_anim

SHOW_FEED = True
SHOW_CONTROL = False
QUEUE_SIZE = 5

if __name__ == "__main__":
    conn = mavutil.mavlink_connection("/dev/ttyUSB0")

    # wait for first heartbeat
    print("Waiting for Heartbeat...")
    conn.wait_heartbeat()
    print(f"Heartbeat from system (system {conn.target_system} component {conn.target_component})")

    """define the required queue"""
    ori_frame_queue = Queue(maxsize=QUEUE_SIZE)
    vis_frame_queue = Queue(maxsize=QUEUE_SIZE)
    control_cmd_queue = Queue(maxsize=QUEUE_SIZE)
    
    """for multiprocessing"""
    if SHOW_CONTROL:
        control_vis_queue = Queue(maxsize=QUEUE_SIZE)

        mp_img_acquisition = Process(target=grab_image, args=(ori_frame_queue,)) 
        mp_obj_detector = Process(target=obj_detector, args=(ori_frame_queue, vis_frame_queue, control_cmd_queue))
        mp_controller = Process(target=gimbal_control_v1, args=(conn, control_cmd_queue, control_vis_queue))
        mp_img_feed = Process(target=image_show, args=(vis_frame_queue, SHOW_FEED))
        mp_control_vis = Process(target=run_anim, args=(control_vis_queue,))

        mp_img_acquisition.start()
        mp_obj_detector.start()
        mp_controller.start()
        mp_img_feed.start()
        mp_control_vis.start()

        mp_img_acquisition.join()
        mp_obj_detector.join()
        mp_controller.join()
        mp_img_feed.join()
        mp_control_vis.join()
    
    else:
        mp_img_acquisition = Process(target=grab_image, args=(ori_frame_queue,)) 
        mp_obj_detector = Process(target=obj_detector, args=(ori_frame_queue, vis_frame_queue, control_cmd_queue))
        mp_controller = Process(target=gimbal_control_v0, args=(conn, control_cmd_queue))
        mp_img_feed = Process(target=image_show, args=(vis_frame_queue, SHOW_FEED))

        mp_img_acquisition.start()
        mp_obj_detector.start()
        mp_controller.start()
        mp_img_feed.start()

        mp_img_acquisition.join()
        mp_obj_detector.join()
        mp_controller.join()
        mp_img_feed.join()