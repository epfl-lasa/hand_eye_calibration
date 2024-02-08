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

def rx_to_angles(R):
    """
    Convert a rotation matrix to ZYX Euler angles (yaw, pitch, and roll).
    
    Args:
        R (numpy.ndarray): A 3x3 rotation matrix.
    
    Returns:
        tuple: A tuple containing the Euler angles (yaw, pitch, roll) in radians.
    """
    assert R.shape == (3, 3), "R must be a 3x3 matrix"

    # Calculate pitch (y rotation)
    pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))

    if math.isclose(pitch, np.pi / 2, abs_tol=1e-3):
        # Gimbal lock, pitch is ~90 degrees
        yaw = math.atan2(R[1, 2], R[1, 1])
        roll = 0
    elif math.isclose(pitch, -np.pi / 2, abs_tol=1e-3):
        # Gimbal lock, pitch is ~-90 degrees
        yaw = math.atan2(-R[1, 2], R[1, 1])
        roll = 0
    else:
        # Calculate yaw (z rotation) and roll (x rotation)
        yaw = math.atan2(R[1, 0], R[0, 0])
        roll = math.atan2(R[2, 1], R[2, 2])

    ratio_degrees = 180/np.pi
    return yaw*ratio_degrees, pitch*ratio_degrees, roll*ratio_degrees

def pose_estimation(A_matrices, B_matrices, method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH):

    n = A_matrices.shape[2]
    
    R_world2cam = []
    t_world2cam = []
    R_base2gripper = []
    t_base2gripper = []
    

    for i in range(n):
        # Extract rotation (R) and translation (t) components for each slice
        R_world2cam.append(A_matrices[:3, :3, i])
        t_world2cam.append(A_matrices[:3, 3, i].reshape(3, 1))
        
        R_base2gripper.append(B_matrices[:3, :3, i])
        t_base2gripper.append(B_matrices[:3, 3, i].reshape(3, 1))

    # Perform hand-eye calibration to solve for the transformation from camera to gripper
    R_base2world, t_base2world, R_gripper2cam, t_gripper2cam = cv2.calibrateRobotWorldHandEye(
        R_world2cam=R_world2cam,
        t_world2cam=t_world2cam,
        R_base2gripper=R_base2gripper,
        t_base2gripper=t_base2gripper,
        method=method
    )

    # Create the 4x4 transformation matrix from the rotation matrix and translation vector
    X = np.eye(4)
    X[:3, :3] = R_base2world
    X[:3, 3] = t_base2world.flatten()
    Y = np.eye(4)
    Y[:3, :3] = R_gripper2cam 
    Y[:3, 3] = t_gripper2cam.flatten()

    return X, Y

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
        
    n = A2.shape[2] 

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
    X1, Y1 = pose_estimation(A1,B1)
    X2, Y2 = pose_estimation(A2,B2)
    
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
    
    errortot1 = 0
    errortottest1 = 0
    errortot2 = 0
    errortottest2 = 0
    errorrot1 = np.zeros(3)
    errorrot1test = np.zeros(3)
    errorrot2 = np.zeros(3)
    errortra1 = np.zeros(3)
    errortra2 = np.zeros(3)
    for i in range(A2.shape[2]):
        errortottest1 = errortottest1 + np.linalg.norm(A1[:,:,i] @ X1 - Y1 @ B1[:,:,i])**2
        errortottest2 = errortottest2 + np.linalg.norm(A2[:,:,i] @ X2 - Y2 @ B2[:,:,i])**2
        errortot1 = errortot1 + np.linalg.norm(A1[:,:,i] @ X_est_batch_BP_1 - Y_est_batch_BP_1 @ B1[:,:,i])**2
        errortot2 = errortot2 + np.linalg.norm(A2[:,:,i] @ X_est_batch_BP_2 - Y_est_batch_BP_2 @ B2[:,:,i])**2
        errorrot1 = errorrot1 + np.power(rx_to_angles((A1[:3,:3,i] @ X_est_batch_BP_1[:3,:3]) @ (Y_est_batch_BP_1[:3,:3] @ B1[:3,:3,i]).T),2)
        errorrot2 = errorrot2 + np.power(rx_to_angles((A2[:3,:3,i] @ X_est_batch_BP_2[:3,:3]) @ (Y_est_batch_BP_2[:3,:3] @ B2[:3,:3,i]).T),2)
        errortra1 = errortra1 + np.power((A1[:3,:3,i] @ X_est_batch_BP_1[:3,3] + A1[:3,3,i])-(Y_est_batch_BP_1[:3,:3] @ B1[:3,3,i] + Y_est_batch_BP_1[:3,3]),2)
        errortra2 = errortra2 + np.power((A2[:3,:3,i] @ X_est_batch_BP_2[:3,3] + A2[:3,3,i])-(Y_est_batch_BP_2[:3,:3] @ B2[:3,3,i] + Y_est_batch_BP_2[:3,3]),2)
        
        
    errortottest1 = np.sqrt(errortottest1) / n
    errortottest2 = np.sqrt(errortottest2) / n
    errortot1 = np.sqrt(errortot1) / n
    errortot2 = np.sqrt(errortot2) / n
    errorrot1 = np.sqrt(errorrot1 / (n-1))
    errorrot2 = np.sqrt(errorrot2 / (n-1))
    errortra1 = np.sqrt(errortra1 / (n-1))
    errortra2 = np.sqrt(errortra2 / (n-1))
    
    print("\nData size:" + str(A2.shape[2]))
    print("\nCase 1 error tot test: \n" + str(errortottest1))
    print("\nCase 1 error tot: \n" + str(errortot1))
    print("\nCase 2 error tot test: \n" + str(errortottest2))
    print("\nCase 2 error tot: \n" + str(errortot2))
    print("\nCase 1 error rot: \n" + str(errorrot1) + "[degrees]")
    print("\nCase 2 error rot: \n" + str(errorrot2) + "[degrees]")
    print("\nCase 1 error tra: \n" + str(errortra1 * 1000) + "[mm]")
    print("\nCase 2 error tra: \n" + str(errortra2 * 1000) + "[mm]")
        
    np.save(os.path.join(script_dir, 'X1_matrices.npy'), X_est_batch_BP_1)
    np.save(os.path.join(script_dir, 'Y1_matrices.npy'), Y_est_batch_BP_1)
    np.save(os.path.join(script_dir, 'X2_matrices.npy'), X_est_batch_BP_2)
    np.save(os.path.join(script_dir, 'Y2_matrices.npy'), Y_est_batch_BP_2)
    
    print("")

if __name__ == '__main__':
    main()
