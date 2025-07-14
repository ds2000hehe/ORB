import cv2
import os
import zmq
import numpy as np


def get_frame():
    zmq_address = 'tcp://localhost:5555'
    context = zmq.context()
    socket = context.socket(zmq.SUB)
    socket.connect(zmq_address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    jpg_bytes = socket.recv()
    np_array = np.frombuffer(jpg_bytes, dtype=np.uint8)
    frame = cv2.imencode(np_array, cv2.IMREAD_COLOR)
    return frame

def main(frame):
    gray =cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mean_intensity = gray.mean()
    return True, mean_intensity
