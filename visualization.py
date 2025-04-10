import os

import cv2

os.environ["XDG_SESSION_TYPE"] = "xcb"

"""resize the image and show it"""
def resize_and_show(frame, show_feed):
    frame_feed = cv2.resize(frame, (int(frame.shape[1]/2),int(frame.shape[0]/2)))
    if show_feed:
        cv2.imshow("Image feed", frame_feed)

"""show the image feed"""
def image_show(frame_queue, show_feed):
    frame_counter, skip_frame = 0, 16
    
    while True:
        frame = frame_queue.get()
        if frame is None:
            break
        
        if frame_counter % skip_frame == 0:
            resize_and_show(frame, show_feed)
        # print(f"Image size: ", frame.shape)
        frame_counter += 1
            
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            return