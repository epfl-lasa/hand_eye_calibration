#!/usr/bin/env python
import sys
import os
import transforms3d
import numpy as np
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

def result_to_se3():
    return 0


def main():
    # Determine the path for loading the .npy files
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    # Load the A and B matrices from .npy files in the same directory as the script
    A1 = np.load(os.path.join(script_dir, 'A1_matrices.npy'))
    B1 = np.load(os.path.join(script_dir, 'B1_matrices.npy'))
    A2 = np.load(os.path.join(script_dir, 'A2_matrices.npy'))
    B2 = np.load(os.path.join(script_dir, 'B2_matrices.npy'))
    n = A.shape[2]//2

    #Iterate over the matrices two by two
    AA1 = np.empty((4, 4, n))
    BB1 = np.empty((4, 4, n))
    AA2 = np.empty((4, 4, n))
    BB2 = np.empty((4, 4, n))
    for i in range(n):
        # Multiply pairs of matrices
        AA1[:, :, i] = A1[:, :, 2*i] @ np.linalg.inv(A1[:, :, 2*i + 1])
        BB1[:, :, i] = B1[:, :, 2*i] @ np.linalg.inv(B1[:, :, 2*i + 1])
        AA2[:, :, i] = A2[:, :, 2*i] @ np.linalg.inv(A2[:, :, 2*i + 1])
        BB2[:, :, i] = B2[:, :, 2*i] @ np.linalg.inv(B2[:, :, 2*i + 1])


    # Senidng data to the Kalman Filtering methods
    ekf1=EKF()
    ekf2=EKF()
    iekf1=IEKF()
    iekf2=IEKF()
    ukf1=UKF()
    ukf2=UKF()
    for i in range(n):
        ekf1.Update(AA1,BB1)
        iekf1.Update(AA1,BB1)
        ukf1.Update(AA1,BB1)
        ekf2.Update(AA2,BB2)
        iekf2.Update(AA2,BB2)
        ukf2.Update(AA2,BB2)
        
    
    # Perform pose estimation
    X_est_batch,Y_est_batch,Y_est_check_batch,ErrorStats_batch = Batch_Processing.pose_estimation(A1, B1)
    
    

    # Print the results
    # Set the numpy print options
    np.set_printoptions(precision=6, suppress=True)
    print('\n.....Batch Processing Results\n')
    print("Y Pose: \n" + str(se3_to_posestamped(Y_est_batch)))
    print("\nX Pose: \n" + str(se3_to_posestamped(X_est_batch)))
    print("\nX Matrix: \n" + str(X_est_batch))
    print("\nErrorStats: " + str(ErrorStats_batch))
    print("")

if __name__ == '__main__':
    main()
