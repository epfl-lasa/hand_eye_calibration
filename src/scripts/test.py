#!/usr/bin/env python
import sys
import os
import math
import pickle
import rospy
import transforms3d
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped

script_dir = os.path.dirname(__file__)
rel_path = "../Pose-Estimation-for-Sensor-Calibration"
abs_file_path = os.path.join(script_dir, rel_path)
sys.path.append(abs_file_path)

# Global variables to store the latest data
latest_cam_pose = None
latest_marker_pose = None

# Callback for the camera topic
def cam_callback(msg):
    global latest_cam_pose
    latest_cam_pose = msg

# Callback for the marker pose
def marker_callback(msg):
    global latest_marker_pose
    latest_marker_pose = msg

def posestamped_to_se3(pose_stamped, inverse_transformation=False):
    """
    Convert PoseStamped message to SO(4) representation (4x4 transformation matrix).
    Optionally, invert the entire transformation matrix.

    Args:
    pose_stamped (geometry_msgs/PoseStamped): PoseStamped message.
    inverse_transformation (bool): If True, invert the entire transformation matrix.

    Returns:
    numpy.ndarray: 4x4 transformation matrix.
    """
    # Extract position
    pos = np.array([pose_stamped.pose.position.x, 
                    pose_stamped.pose.position.y, 
                    pose_stamped.pose.position.z])

    # Extract orientation (quaternion [w, x, y, z] used by transform3d)
    quat = [pose_stamped.pose.orientation.w, 
            pose_stamped.pose.orientation.x, 
            pose_stamped.pose.orientation.y, 
            pose_stamped.pose.orientation.z]

    # Convert quaternion to 3x3 rotation matrix
    rotation_matrix = transforms3d.quaternions.quat2mat(quat)

    # Construct the 4x4 transformation matrix
    transformation_matrix = np.zeros((4, 4))
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = pos
    transformation_matrix[3, 3] = 1
    
    # Invert the entire transformation matrix if requested
    if inverse_transformation:
        return np.linalg.inv(transformation_matrix)

    return transformation_matrix

    T = result.x[3:]
    rotation = result.x[:3]
    
    angle = np.linalg.norm(rotation)
    if angle < 1e-6:  # Using a small epsilon value for float comparison
        k = [0, 1, 0]  # Default axis if angle is too small
    else:
        k = rotation / angle  # Normalize the axis vector

    # Calculate the rotation matrix using Rodrigues' rotation formula
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

    # Create a 4x4 homogeneous transformation matrix
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = T
    return M

def main():
    # Determine the path for loading the .npy files
    script_dir = os.path.dirname(os.path.realpath(__file__))
    rospy.init_node('calibration_tester')
    
    rospy.Subscriber("/vrpn_client_node/cam_grasp/pose", PoseStamped, cam_callback)
    rospy.Subscriber("aruco_single/pose", PoseStamped, marker_callback)
    
    # Load the X and Y matrices from .npy files in the same directory as the script
    X1 = np.load(os.path.join(script_dir, 'X1_matrices.npy'))
    Y1 = np.load(os.path.join(script_dir, 'Y1_matrices.npy'))
    X2 = np.load(os.path.join(script_dir, 'X2_matrices.npy'))
    Y2 = np.load(os.path.join(script_dir, 'Y2_matrices.npy'))
    
    time_threshold = 0.5
    try:
        print("Starting recording process")
        while not rospy.is_shutdown():
            # Store and print the latest data
            if latest_cam_pose and  latest_marker_pose:
                now = rospy.Time.now().to_sec()
                delta_time_camera =  now - latest_cam_pose.header.stamp.to_sec()
                delta_time_marker = now - latest_marker_pose.header.stamp.to_sec()
                
                if np.max((delta_time_camera, delta_time_marker)) < time_threshold:
                    A1 = posestamped_to_se3(latest_cam_pose)
                    B1 = posestamped_to_se3(latest_marker_pose, inverse_transformation = True)
                    Y1 = A1 @ X1 @ np.linalg.inv(B1)
                    print(Y1[:3,3])
                else:
                    if delta_time_camera > time_threshold:
                        rospy.logerr("No data from camera pose since [s]: " +str(delta_time_camera))
                    if delta_time_marker > time_threshold:
                        rospy.logerr("No data from marker pose since [s]: " +str(delta_time_marker))
                        
                        
            else:
                if not latest_cam_pose:
                    rospy.logerr("No data received from camera pose topic")
                if not latest_marker_pose:
                    rospy.logerr("No data received from marker pose topic")
                    
            rospy.sleep(1)
            
    except rospy.ROSInterruptException:
        pass
    
    
if __name__ == '__main__':
    main()
