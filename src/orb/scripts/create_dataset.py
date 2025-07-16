#!/usr/bin/env python3

import cv2
import os
import zmq
import numpy as np
from datetime import datetime
import time


SAVE_DIR = "/home/david/gaussian_spitting_data"
CAPTURE_INTERVAL = 3


def connect_frame():
    zmq_address = 'tcp://localhost:5555'
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(zmq_address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    return socket

def get_frame(socket):
    jpg_bytes = socket.recv()
    np_array = np.frombuffer(jpg_bytes, dtype=np.uint8)
    frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
    return frame

def main():
    try:
        while True:
            socket = connect_frame()
            frame = get_frame(socket)
            generate_dataset(frame, SAVE_DIR)
            time.sleep(CAPTURE_INTERVAL)
    except KeyboardInterrupt:
        print('stop')
    finally:
        cv2.destroyAllWindows()


def test():
    socket = connect_frame()
    frame = get_frame(socket)
    mean_intensity = frame.mean()
    cv2.resize(frame, (1920, 1080), interpolation=cv2.INTER_LINEAR)
    cv2.imshow('captured frame', frame)
    cv2.waitKey(5000)

def generate_dataset(frame, save_dir):

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(save_dir, f"frame_{timestamp}.jpg")
    cv2.imwrite(filename, frame)

cv2.destroyAllWindows()


   
if __name__ == '__main__':
    main()
