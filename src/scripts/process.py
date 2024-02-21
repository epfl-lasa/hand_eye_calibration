#!/usr/bin/env python
import os
import math
import numpy as np
import cv2

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
    A1 = np.load(os.path.join(script_dir, 'A1_matrices.npy'))
    B1 = np.load(os.path.join(script_dir, 'B1_matrices.npy'))
    A2 = np.load(os.path.join(script_dir, 'A2_matrices.npy'))
    B2 = np.load(os.path.join(script_dir, 'B2_matrices.npy'))
        
    n = A2.shape[2] 
    
    # Perform pose estimation
    X1, Y1 = pose_estimation(A1,B1)
    X2, Y2 = pose_estimation(A2,B2)

    # Print the results
    # Set the numpy print options
    np.set_printoptions(precision=6, suppress=True)
    print("\nX1 Matrix: \n" + str((X1)))
    print("\nY1 Matrix: \n" + str((Y1)))
    print("\nX2 Matrix: \n" + str((X2)))
    print("\nY2 Matrix: \n" + str((Y2)))
    print("")
    
    e_tot_1 = 0
    e_tot_2 = 0
    e_rot_1 = np.zeros(3)
    e_rot_2 = np.zeros(3)
    e_tra_1 = np.zeros(3)
    e_tra_2 = np.zeros(3)
    for i in range(A2.shape[2]):
        e_tot_1 = e_tot_1 + np.linalg.norm(A1[:,:,i] @ X1 - Y1 @ B1[:,:,i])**2
        e_tot_2 = e_tot_2 + np.linalg.norm(A2[:,:,i] @ X2 - Y2 @ B2[:,:,i])**2
        e_rot_1 = e_rot_1 + np.absolute(rx_to_angles((A1[:3,:3,i] @ X1[:3,:3]) @ (Y1[:3,:3] @ B1[:3,:3,i]).T))
        e_rot_2 = e_rot_2 + np.absolute(rx_to_angles((A2[:3,:3,i] @ X2[:3,:3]) @ (Y2[:3,:3] @ B2[:3,:3,i]).T))
        e_tra_1 = e_tra_1 + np.absolute((A1[:3,:3,i] @ X1[:3,3] + A1[:3,3,i])-(Y1[:3,:3] @ B1[:3,3,i] + Y1[:3,3]))
        e_tra_2 = e_tra_2 + np.absolute((A2[:3,:3,i] @ X2[:3,3] + A2[:3,3,i])-(Y2[:3,:3] @ B2[:3,3,i] + Y2[:3,3]))
        
        

        
    e_tot_1 = np.sqrt(e_tot_1) / n
    e_tot_2 = np.sqrt(e_tot_2) / n
    e_rot_1 = e_rot_1 / n
    e_rot_2 = e_rot_2 / n
    e_tra_1 = e_tra_1 / n
    e_tra_2 = e_tra_2 / n
    
    print("\nData size:" + str(A2.shape[2]))
    print("\nCase 1 error tot: \n" + str(e_tot_1))
    print("\nCase 2 error tot: \n" + str(e_tot_2))
    print("\nCase 1 error rot: \n" + str(e_rot_1) + "[degrees]")
    print("\nCase 2 error rot: \n" + str(e_rot_2) + "[degrees]")
    print("\nCase 1 error tra: \n" + str(e_tra_1 * 1000) + "[mm]")
    print("\nCase 2 error tra: \n" + str(e_tra_2 * 1000) + "[mm]")
        
    np.save(os.path.join(script_dir, 'X1_matrices.npy'), X1)
    np.save(os.path.join(script_dir, 'Y1_matrices.npy'), Y1)
    np.save(os.path.join(script_dir, 'X2_matrices.npy'), X2)
    np.save(os.path.join(script_dir, 'Y2_matrices.npy'), Y2)
    
    print("")

if __name__ == '__main__':
    main()
