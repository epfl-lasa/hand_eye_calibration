#!/usr/bin/env python
import sys
import os
import rospy
import transforms3d
import numpy as np
from geometry_msgs.msg import PoseStamped

# Global variables to store the latest data
latest_cam_grasp_pose = None
latest_aruco_pose = None

# Callback for the /vrpn_client_node/cam_grasp/pose_transform topic
def cam_grasp_callback(msg):
    global latest_cam_grasp_pose
    latest_cam_grasp_pose = msg

# Callback for the aruco_single/pose topic
def aruco_callback(msg):
    global latest_aruco_pose
    latest_aruco_pose = msg

def posestamped_to_so4(pose_stamped, inverse_transformation=False):
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

def main():
    # Initialize the ROS Node
    rospy.init_node('pose_subscriber', anonymous=True)

    # Subscribe to the topics
    rospy.Subscriber("/vrpn_client_node/cam_grasp/pose", PoseStamped, cam_grasp_callback)
    rospy.Subscriber("aruco_single/pose", PoseStamped, aruco_callback)

    n = 20 #1+2*6 # First centered then pos/neg translation/rotation for each axis
    A = np.zeros((4,4,n))
    B = np.zeros((4,4,n))
    
    i = 0
    
    
    # Main loop
    try:
        while not rospy.is_shutdown() and i < n:
            input("Press Enter to save the current poses...")
            print("")
            
            # Store and print the latest data
            if latest_cam_grasp_pose and latest_aruco_pose:
                now = rospy.Time.now().to_sec()
                delta_time_optitrack = latest_cam_grasp_pose.header.stamp.to_sec() - now
                delta_time_marker = latest_aruco_pose.header.stamp.to_sec() - now
                if delta_time_optitrack < 0.5 and delta_time_marker < 0.5:
                    A[:,:,i]= posestamped_to_so4(latest_cam_grasp_pose)
                    B[:,:,i]= posestamped_to_so4(latest_aruco_pose, inverse_transformation = True)
                    i+=1
                else:
                    if delta_time_optitrack > 0.5:
                        print("No data from optitrack since [s]: " +str(delta_time_optitrack))
                    if delta_time_marker > 0.5:
                        print("No data from marker since [s]: " +str(delta_time_optitrack))
                        
            else:
                if latest_cam_grasp_pose:
                    print("No data received from marker pose")
                elif latest_aruco_pose:
                    print("No data received from optitrack for camera pose")
                else:
                    print("No data received")
            
            print("Picture " +str(i) + "/" + str(n))
            rospy.sleep(1) 
            
                    

    except rospy.ROSInterruptException:
        pass
    
    # Determine the path for saving the .npy files
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    # Save the A and B matrices to .npy files in the same directory as the script
    np.save(os.path.join(script_dir, 'A_matrices.npy'), A)
    np.save(os.path.join(script_dir, 'B_matrices.npy'), B)


if __name__ == '__main__':
    main()
