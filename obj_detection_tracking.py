import os
import time
from random import randrange
from collections import Counter 

import numpy as np
import cv2
# from ultralytics import YOLO
# from SFSORT import SFSORT

weight_path = "/home/wicomai/occ/model/best_9t_int8.engine"

"""
Execute AI model for object detection
Find and extract the RoI of OOK and OFDM LED
Put each detected RoI to each queue (OOK and OFDM)
"""
def obj_detector(frame_queue, vis_queue):
    """load the object detection model"""
    # model = YOLO(weight_path, task="detect") #load the model, TODO: update model
    
    # tracker_arguments = {"dynamic_tuning": True, "cth": 0.5,
    #                   "high_th": 0.6, "high_th_m": 0.1,
    #                   "match_th_first": 0.67, "match_th_first_m": 0.05,
    #                   "match_th_second": 0.2, "low_th": 0.1,
    #                   "new_track_th": 0.7, "new_track_th_m": 0.1,
    #                   "marginal_timeout": (7 * 220 // 10),
    #                   "central_timeout": 220,
    #                   "horizontal_margin": 720 // 10,
    #                   "vertical_margin": 590 // 10,
    #                   "frame_width": 720,
    #                   "frame_height": 590}
    
    # tracker = SFSORT(tracker_arguments)
    
    colors = {}
    start = True
    obj_tx = {}
    ofdm_frames = {}
    ook_frames = {}
    
    """for measuring fps"""
    fps_timer_start = time.time()
    frame_counter = 0
    
    """continuously grab fresh image from queue"""
    while True:
        """get frame from original sampled image queue"""
        frame = frame_queue.get()
        # show_frame = frame.copy()
        
        """resize frame for object detection model"""
        # frame = cv2.resize(frame, (frame.shape[1]//2, frame.shape[0]//2))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # print(f"Resized frame shape: {frame.shape}")
        
        """extract the OFDM RoI manually"""
        ofdm_frame = frame[365:475,390:502] #row,col, 
        """extract the OOK RoI manually"""
        # ook_frame = frame[365:475,603:715] #row,col, 
        ook_frame = frame[365:475,390:502] #row,col
        
        """put the object RoI and frame to its queue"""
        vis_queue.put(frame) #put the frame with RoI bbox for visualization
        
        """For FPS calculation"""
        frame_counter += 1
        cur_time = time.time()
        if cur_time - fps_timer_start > 1:
            print(f"FPS det: {frame_counter / (cur_time - fps_timer_start)}")
            frame_counter = 0
            fps_timer_start = time.time()