import os
import datetime
from multiprocessing import Process, Queue

import cv2
import numpy as np

from ct_cam_init import grab_image
from obj_detection_tracking import obj_detector
from visualization import image_show

if __name__ == "__main__":
    """define the required queue"""
    ori_frame_queue = Queue(maxsize=5000)
    vis_frame_queue = Queue(maxsize=5000)
    show_feed = True
    
    """for multiprocessing"""
    mp_img_acquisition = Process(target=grab_image, args=(ori_frame_queue,)) 
    mp_obj_detector = Process(target=obj_detector, args=(ori_frame_queue,vis_frame_queue))
    mp_img_feed = Process(target=image_show, args=(vis_frame_queue,show_feed))
    
    mp_img_acquisition.start()
    mp_obj_detector.start()
    mp_img_feed.start()
    
    mp_img_acquisition.join()
    mp_obj_detector.join()
    mp_img_feed.join()