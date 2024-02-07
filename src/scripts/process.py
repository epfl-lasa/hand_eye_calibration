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

def main():
    # Determine the path for loading the .npy files
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    # Load the A and B matrices from .npy files in the same directory as the script
    real_data = True
    if real_data:
        A1 = np.load(os.path.join(script_dir, 'A1_matrices.npy'))
        B1 = np.load(os.path.join(script_dir, 'B1_matrices.npy'))
        A2 = np.load(os.path.join(script_dir, 'A2_matrices.npy'))
        B2 = np.load(os.path.join(script_dir, 'B2_matrices.npy'))
    else:
        # Determine the directory of the current file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Construct the path to the data file in the same directory
        data_file = os.path.join(current_dir, '../Pose-Estimation-for-Sensor-Calibration/pose_sim_data_noisy.p') #random 3deg, 3mm noise added to measurements
        with open(data_file, mode='rb') as f:
            sim_data = pickle.load(f)
        A1=sim_data['xfm_A']
        B1=sim_data['xfm_B']
        A2=A1[:,:,:]
        B2=B1[:,:,:]
        
    n = A2.shape[2] - 1

    #Iterate over the matrices two by two
    # AA1 = np.empty((4, 4, n))
    # BB1 = np.empty((4, 4, n))
    # AA2 = np.empty((4, 4, n))
    # BB2 = np.empty((4, 4, n))
    
    
    # Senidng data to the Kalman Filtering methods
    # ekf1=EKF()
    # iekf1=IEKF()
    # ukf1=UKF()
    # ekf2=EKF()
    # iekf2=IEKF()
    # ukf2=UKF()
    # for i in range(n):
        # Multiply pairs of matrices
        # AA1[:, :, i] = np.linalg.inv(A1[:, :, i]) @ A1[:, :, i + 1]
        # BB1[:, :, i] = np.linalg.inv(B1[:, :, i]) @ B1[:, :, i + 1]
        # AA2[:, :, i] = np.linalg.inv(A2[:, :, i]) @ A2[:, :, i + 1]
        # BB2[:, :, i] = np.linalg.inv(B2[:, :, i]) @ B2[:, :, i + 1]
        # ekf1.Update(AA1[:, :, i],BB1[:, :, i])
        # iekf1.Update(AA1[:, :, i],BB1[:, :, i])
        # ukf1.Update(AA1[:, :, i],BB1[:, :, i])
        # ekf2.Update(AA2[:, :, i],BB2[:, :, i])
        # iekf2.Update(AA2[:, :, i],BB2[:, :, i])
        # ukf2.Update(AA2[:, :, i],BB2[:, :, i])
        # print("{:.3f}%".format(100*(i+1)/n))

    
    # Perform pose estimation
    X_est_batch_BP_1,Y_est_batch_BP_1, _, _ = Batch_Processing.pose_estimation(A1, B1)
    X_est_batch_BP_2,Y_est_batch_BP_2, _, _ = Batch_Processing.pose_estimation(A2, B2)
    
    
    # X_est_batch_EKF_1 = result_to_se3(ekf1)
    # X_est_batch_IEKF_1 = result_to_se3(iekf1)
    # X_est_batch_UKF_1 = result_to_se3(ukf1)
    # X_est_batch_EKF_2 = result_to_se3(ekf2)
    # X_est_batch_IEKF_2 = result_to_se3(iekf2)
    # X_est_batch_UKF_2 = result_to_se3(ukf2)
    
    

    # Print the results
    # Set the numpy print options
    np.set_printoptions(precision=6, suppress=True)
    print("\nX Matrix BP1: \n" + str((X_est_batch_BP_1)))
    # print("\nX Matrix: \n" + str(X_est_batch_EKF_1))
    # print("\nX Matrix: \n" + str(X_est_batch_IEKF_1))
    # print("\nX Matrix: \n" + str(X_est_batch_UKF_1))
    print("\nY Matrix BP1: \n" + str((Y_est_batch_BP_1)))
    print("\nX Matrix BP2: \n" + str((X_est_batch_BP_2)))
    # print("\nX Matrix EKF2: \n" + str(X_est_batch_EKF_2))
    # print("\nX Matrix IEKF2: \n" + str(X_est_batch_IEKF_2))
    # print("\nX Matrix UKF2: \n" + str(X_est_batch_UKF_2))
    print("\nY Matrix BP2: \n" + str((Y_est_batch_BP_2)))
    print("")
    
    est_X_BP1=np.empty((4, 4, n))
    est_X_BP2=np.empty((4, 4, n))
    est_Y_BP1=np.empty((4, 4, n))
    est_Y_BP2=np.empty((4, 4, n))
    # est_EKF=np.empty((4, 4, n))
    # est_IEKF=np.empty((4, 4, n))
    # est_UKF=np.empty((4, 4, n))
    # est_EKF2=np.empty((4, 4, n))
    for i in range(A2.shape[2]-1):
        est_X_BP1[:,:,i]= np.linalg.inv(A1[:,:,i]) @ Y_est_batch_BP_1 @ B1[:,:,i]
        est_X_BP2[:,:,i]= np.linalg.inv(A2[:,:,i]) @ Y_est_batch_BP_2 @ B2[:,:,i]
        est_Y_BP1[:,:,i]= A1[:,:,i] @ X_est_batch_BP_1 @ np.linalg.inv(B1[:,:,i])
        est_Y_BP2[:,:,i]= A2[:,:,i] @ X_est_batch_BP_2 @ np.linalg.inv(B2[:,:,i])
        # est_EKF2[:,:,i] = np.linalg.inv(AA2[:,:,i]) @ X_est_batch_EKF_2 @ BB2[:,:,i]
        # est_IEKF[:,:,i] = np.linalg.inv(AA2[:,:,i]) @ X_est_batch_IEKF_2 @ BB2[:,:,i]
        # est_UKF[:,:,i] = np.linalg.inv(AA2[:,:,i]) @ X_est_batch_UKF_2 @ BB2[:,:,i]
        
    print("\nX Matrix BP1 std dev: \n" + str(np.std(est_X_BP1, axis=2)))
    print("\nY Matrix BP1 std dev: \n" + str(np.std(est_Y_BP1, axis=2)))
    print("\nX Matrix BP2 std dev: \n" + str(np.std(est_X_BP2, axis=2)))
    print("\nY Matrix BP2 std dev: \n" + str(np.std(est_Y_BP2, axis=2)))
    # print(np.std(est_EKF, axis=2))
    # print(np.std(est_IEKF, axis=2))
    # print(np.std(est_UKF, axis=2))
    # print(np.std(est_EKF2, axis=2))
        
    np.save(os.path.join(script_dir, 'X1_matrices.npy'), X_est_batch_BP_1)
    np.save(os.path.join(script_dir, 'Y1_matrices.npy'), Y_est_batch_BP_1)
    np.save(os.path.join(script_dir, 'X2_matrices.npy'), X_est_batch_BP_2)
    np.save(os.path.join(script_dir, 'Y2_matrices.npy'), Y_est_batch_BP_2)
    
    print("")

if __name__ == '__main__':
    main()
