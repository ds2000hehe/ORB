import os
import cv2
import numpy as np
import random
import matplotlib
from cv_bridge import CvBridge
from img_receive import get_frame, main
import time


ROS_running = True
K_matrix = np.load("camera_intrinsics.npy")

def __init__(self):
    self.K = np.array([[718]])

    self.start_time = time.time()
    self.orb = cv2.ORB_create()
    self.ROS_running = True

def get_keypoints(self):
    img = get_frame()
    timestamp = time.time() - self.start_time

    keypoints, descriptors = self.orb.detectAndCompute(img, None)
    return keypoints, descriptors

def match_descr(desc1, desc2):

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(desc1, desc2)
    matches = sorted(matches, key=lambda x: x.distance)

    return [m for m in matches if m.distance < 30]
    

def estimate_motion(kp1, kp2, matches, K):

    p1 = np.float32([kp1[m.queryIdx].pt for m in matches])
    p2 = np.float32([kp2[m.queryIdx].pt for m in matches])

    E, mask = cv2.findEssentialMat(p1, p2, K, methd=cv2.RANSAC, prob=0.999, threshold=1.0)
    _, R, t, mask_pose = cv2.recoverPose(E, p1, p2, K, mask=mask)
    return R, t, p1[mask.ravel() == 1], p2[mask.ravel() == 1]


def triangulate_points(P1, P2, p1, p2):
    p4 = cv2.triangulatePoints(P1, P2, p1.T, p2.T)
    p3 = p4[:3] / p4[3]
    return p3.T

def main(self):
    
    init_pose = np.eye(3)
    init_pos = np.zeros((3, 1))

    pose_list = [init_pose.copy(), init_pos.copy()]

    ret, frame = main(get_frame())

    if not ret:
        print('Frame not revieved')
        return
    
    kp1, des1 = get_keypoints(frame)
    frame_idx = 1

    while True:
        ret, frame = main(get_frame())
        if not ret:
            break

        kp2, des2 = get_keypoints(frame)
        frame_idx = 1

        matches = self.match_descr(des1, des2)

        if len(matches) < 8:
            print("not enough matches")
            continue

        R, t, matched_pts1, matched_pts2 = estimate_motion(kp1, kp2, matches, K_matrix)

        P1 = K_matrix @ np.hstack((np.eye(3), np.zeros((3, 1))))
        P2 = K_matrix @ np.hstack((R,t))
        pts_3d = triangulate_points(P1, P2, matched_pts1, matched_pts2)
        
        cur_t += cur_R @ t
        cur_R = R @ cur_R
        pose_list.append((cur_R.copy(), cur_t.copy()))

        kp1, des1 = kp2, des2
        frame = frame.copy()
        frame_idx += 1





    



