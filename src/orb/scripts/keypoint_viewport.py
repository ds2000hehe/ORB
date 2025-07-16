#!/usr/bin/env python3

import cv2
import zmq
import numpy as np

def get_frame():
    zmq_address = 'tcp://localhost:5555'
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(zmq_address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    jpg_bytes = socket.recv()
    np_array = np.frombuffer(jpg_bytes, dtype=np.uint8)
    frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
    return frame

def main():
    orb = cv2.ORB_create()

    while True:
        try:
            frame = get_frame()
            keypoints = orb.detect(frame, None)
            frame_with_kp = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0))

            cv2.imshow("ORB Keypoints", frame_with_kp)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
            continue

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()