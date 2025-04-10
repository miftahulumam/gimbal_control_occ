import os
import datetime
from multiprocessing import Process, Queue

import cv2
import numpy as np
from pymavlink import mavutil

from ct_cam_init import grab_image
from obj_det_gimbal_tracking import obj_detector
from gimbal_control_ct import gimbal_control
from visualization import image_show

if __name__ == "__main__":
    conn = mavutil.mavlink_connection("/dev/ttyUSB0")

    # wait for first heartbeat
    print("Waiting for Heartbeat...")
    conn.wait_heartbeat()
    print(f"Heartbeat from system (system {conn.target_system} component {conn.target_component})")

    """define the required queue"""
    ori_frame_queue = Queue(maxsize=1)
    vis_frame_queue = Queue(maxsize=1)
    control_cmd_queue = Queue(maxsize=1)
    show_feed = True
    
    """for multiprocessing"""
    mp_img_acquisition = Process(target=grab_image, args=(ori_frame_queue,)) 
    mp_obj_detector = Process(target=obj_detector, args=(ori_frame_queue,vis_frame_queue, control_cmd_queue))
    mp_controller = Process(target=gimbal_control, args=(conn, control_cmd_queue))
    mp_img_feed = Process(target=image_show, args=(vis_frame_queue,show_feed))
    
    mp_img_acquisition.start()
    mp_obj_detector.start()
    mp_controller.start()
    mp_img_feed.start()
    
    mp_img_acquisition.join()
    mp_obj_detector.join()
    mp_controller.join()
    mp_img_feed.join()