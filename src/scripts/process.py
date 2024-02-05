#!/usr/bin/env python
import sys
import os
import math
import pickle
import transforms3d
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped

script_dir = os.path.dirname(__file__)
rel_path = "../Pose-Estimation-for-Sensor-Calibration"
abs_file_path = os.path.join(script_dir, rel_path)
sys.path.append(abs_file_path)

from Pose_Estimation_Class import Batch_Processing, EKF, IEKF, UKF

def se3_to_posestamped(transformation_matrix):
    """
    Convert a 4x4 transformation matrix (SE(3)) to a PoseStamped message.

    Args:
    transformation_matrix (numpy.ndarray): 4x4 transformation matrix.

    Returns:
    geometry_msgs/PoseStamped: PoseStamped message.
    """
    # Initialize PoseStamped message
    pose_stamped = PoseStamped()

    # Extract the rotation matrix and translation vector from the transformation matrix
    rotation_matrix = transformation_matrix[:3, :3]
    translation_vector = transformation_matrix[:3, 3]

    # Convert rotation matrix to quaternion
    quat = transforms3d.quaternions.mat2quat(rotation_matrix)

    # Assign position
    pose_stamped.pose.position.x = translation_vector[0]
    pose_stamped.pose.position.y = translation_vector[1]
    pose_stamped.pose.position.z = translation_vector[2]

    # Assign orientation
    pose_stamped.pose.orientation.w = quat[0]
    pose_stamped.pose.orientation.x = quat[1]
    pose_stamped.pose.orientation.y = quat[2]
    pose_stamped.pose.orientation.z = quat[3]

    return pose_stamped

def result_to_se3(result):
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

def pose_estimation(A_matrices, B_matrices, method=cv2.CALIB_HAND_EYE_TSAI):
    """
    Estimate the pose transformation between a robot's end-effector and a camera using hand-eye calibration,
    given 3D arrays of transformation matrices.
    
    Args:
    - A_matrices (np.array): A 3D numpy array of shape (4, 4, n) representing the robot base to end-effector transformations.
    - B_matrices (np.array): A 3D numpy array of shape (4, 4, n) representing the camera to calibration object transformations.
    
    Returns:
    - transformation_matrix (np.array): The 4x4 transformation matrix from the camera to the robot gripper.
    """
    
    n = A_matrices.shape[2]
    R_gripper2base_vec = []
    t_gripper2base = []
    R_target2cam_vec = []
    t_target2cam = []
    
    for i in range(n):
        # Extract rotation (R) and translation (t) components for each slice
        R_gripper2base_vec.append(A_matrices[:3, :3, i])
        t_gripper2base.append(A_matrices[:3, 3, i].reshape(3, 1))

        R_target2cam_vec.append(B_matrices[:3, :3, i])
        t_target2cam.append(B_matrices[:3, 3, i].reshape(3, 1))
    
    # Perform hand-eye calibration to solve for the transformation from camera to gripper
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base=R_gripper2base_vec,
        t_gripper2base=t_gripper2base,
        R_target2cam=R_target2cam_vec,
        t_target2cam=t_target2cam,
        method=method
    )

    # Create the 4x4 transformation matrix from the rotation matrix and translation vector
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R_cam2gripper
    transformation_matrix[:3, 3] = t_cam2gripper.flatten()

    return transformation_matrix

def main():
    # Determine the path for loading the .npy files
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    # Load the A and B matrices from .npy files in the same directory as the script
    real_data = False
    if real_data:
        A1 = np.load(os.path.join(script_dir, 'A1_matrices.npy'))
        B1 = np.load(os.path.join(script_dir, 'B1_matrices.npy'))
        A2 = np.load(os.path.join(script_dir, 'A2_matrices.npy'))
        B2 = np.load(os.path.join(script_dir, 'B2_matrices.npy'))
    else:
        # Determine the directory of the current file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Construct the path to the data file in the same directory
        data_file = os.path.join(current_dir, '../Pose-Estimation-for-Sensor-Calibration/pose_sim_data.p') #random 3deg, 3mm noise added to measurements
        with open(data_file, mode='rb') as f:
            sim_data = pickle.load(f)
        A1=sim_data['xfm_A']
        B1=sim_data['xfm_B']
        A2=A1
        B2=B1
        
    n = A1.shape[2]-1

    #Iterate over the matrices two by two
    AA1 = np.empty((4, 4, n))
    BB1 = np.empty((4, 4, n))
    AA2 = np.empty((4, 4, n))
    BB2 = np.empty((4, 4, n))
    
    
    # Senidng data to the Kalman Filtering methods
    ekf1=EKF()
    ekf2=EKF()
    iekf1=IEKF()
    iekf2=IEKF()
    ukf1=UKF()
    ukf2=UKF()
    for i in range(n):
        # Multiply pairs of matrices
        AA1[:, :, i] = np.linalg.inv(A1[:, :, i]) @ A1[:, :, i + 1]
        BB1[:, :, i] = np.linalg.inv(B1[:, :, i]) @ B1[:, :, i + 1]
        # AA2[:, :, i] = A2[:, :, i] @ np.linalg.inv(A2[:, :, i + 1])
        # BB2[:, :, i] = B2[:, :, i] @ np.linalg.inv(B2[:, :, i + 1])
        # ekf1.Update(AA1[:, :, i],BB1[:, :, i])
        # iekf1.Update(AA1[:, :, i],BB1[:, :, i])
        # ukf1.Update(AA1[:, :, i],BB1[:, :, i])
        # ekf2.Update(AA2[:, :, i],BB2[:, :, i])
        # iekf2.Update(AA2[:, :, i],BB2[:, :, i])
        # ukf2.Update(AA2[:, :, i],BB2[:, :, i])
        print("{:.3f}%".format((i+1)/n))

    
    # Perform pose estimation
    X_est_batch_BP_1,Y_est_batch_BP_1, _, _ = Batch_Processing.pose_estimation(A1, B1)
    X_est = pose_estimation(A1,B1, method=cv2.CALIB_HAND_EYE_TSAI)
    X_est2 = pose_estimation(A1,B1, method=cv2.CALIB_HAND_EYE_PARK)
    X_est3 = pose_estimation(A1,B1, method=cv2.CALIB_HAND_EYE_HORAUD)
    X_est4 = pose_estimation(A1,B1, method=cv2.CALIB_HAND_EYE_ANDREFF)
    # X_est_batch_BP_2,Y_est_batch_BP_2, _, _ = Batch_Processing.pose_estimation(A2, B2)
    
    
    # X_est_batch_EKF_1 = result_to_se3(ekf1)
    # X_est_batch_IEKF_1 = result_to_se3(iekf1)
    # X_est_batch_UKF_1 = result_to_se3(ukf1)
    
    

    # Print the results
    # Set the numpy print options
    np.set_printoptions(precision=6, suppress=True)
    print("\nX Matrix: \n" + str(se3_to_posestamped(X_est_batch_BP_1)))
    print("\nX Matrix: \n" + str(se3_to_posestamped(X_est)))
    print("\nX Matrix: \n" + str(se3_to_posestamped(X_est2)))
    print("\nX Matrix: \n" + str(se3_to_posestamped(X_est3)))
    print("\nX Matrix: \n" + str(se3_to_posestamped(X_est4)))
    # print("\nX Matrix: \n" + str(X_est_batch_EKF_1))
    # print("\nX Matrix: \n" + str(X_est_batch_IEKF_1))
    # print("\nX Matrix: \n" + str(X_est_batch_UKF_1))
    
    #print('\n.....Batch Processing Results\n')
    #print("Y Pose: \n" + str(se3_to_posestamped(Y_est_batch_BP_1)))
    #print("\nX Pose: \n" + str(se3_to_posestamped(X_est_batch_BP_1)))
    #print("\nX Matrix: \n" + str(X_est_batch_BP_1))
    print("")

if __name__ == '__main__':
    main()
