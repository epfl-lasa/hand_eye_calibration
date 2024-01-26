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

from Pose_Estimation_Class import Batch_Processing

def so4_to_posestamped(transformation_matrix):
    """
    Convert a 4x4 transformation matrix (SO(4)) to a PoseStamped message.

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

def main():
    # Determine the path for loading the .npy files
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    # Load the A and B matrices from .npy files in the same directory as the script
    A = np.load(os.path.join(script_dir, 'A_matrices.npy'))
    B = np.load(os.path.join(script_dir, 'B_matrices.npy'))

    # Perform pose estimation
    X_est,Y_est,Y_est_check,ErrorStats = Batch_Processing.pose_estimation(A, B)

    # Print the results
    # Set the numpy print options
    np.set_printoptions(precision=6, suppress=True)
    print('\n.....Batch Processing Results\n')
    print("Y Pose: \n" + str(so4_to_posestamped(Y_est)))
    print("\nX Pose: \n" + str(so4_to_posestamped(X_est)))
    print("\nX Matrix: \n" + str(X_est))
    print("\nErrorStats: " + str(ErrorStats))
    print("")

if __name__ == '__main__':
    main()
