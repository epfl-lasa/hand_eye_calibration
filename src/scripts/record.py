#!/usr/bin/env python
import sys
import os
import rospy
import transforms3d
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# Global variables to store the latest data
latest_cam_pose = None
latest_robot_base_pose = None
latest_marker_pose = None
latest_ee_pose = None
recording = None

# Callback for the camera topic
def cam_callback(msg):
    global latest_cam_pose
    latest_cam_pose = msg
    
# Callback for the robot base topic
def robot_base_callback(msg):
    global latest_robot_base_pose
    latest_robot_base_pose = msg

# Callback for the marker pose
def marker_callback(msg):
    global latest_marker_pose
    latest_marker_pose = msg

# Callback for the marker pose
def ee_pose_callback(msg):
    global latest_ee_pose
    latest_ee_pose = msg

# Boolean callback to start recording pose data (robot reached correct pose) 
def recording_callback(msg):
    global recording 
    recording = msg.data

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

def main():
    global latest_cam_pose, latest_robot_base_pose, latest_marker_pose, latest_ee_pose, recording
    # Initialize the ROS Node
    #rospy.init_node('calibration_recorder', anonymous=True)
    rospy.init_node('calibration_recorder')

    # Subscribe to the topics
    rospy.Subscriber("/vrpn_client_node/cam_grasp/pose", PoseStamped, cam_callback)
    rospy.Subscriber("/vrpn_client_node/franka_base16/pose", PoseStamped, robot_base_callback)
    rospy.Subscriber("aruco_single/pose", PoseStamped, marker_callback)
    rospy.Subscriber("calibration/ee_pose", PoseStamped, ee_pose_callback)
    rospy.Subscriber("calibration/recording", Bool, recording_callback)

    recorded_pub = rospy.Publisher("calibration/position_recorded", Bool, queue_size=1)

    n = 3**6 # depends on the number of poses in the control algorithm
    A1 = np.zeros((4,4,n))
    B1 = np.zeros((4,4,n))
    A2 = np.zeros((4,4,n))
    B2 = np.zeros((4,4,n))
    
    i = 0
    
    time_threshold = 0.5 #Ensures that we are still receiving data from optitrack
    
    # Main loop
    try:
        print("Starting recording process")
        while not rospy.is_shutdown() and i < n:
            if recording:
                rospy.sleep(2*time_threshold) #wait for arm to stop moving
                data_captured = False
                # Store and print the latest data
                if latest_cam_pose and latest_robot_base_pose and latest_marker_pose and latest_ee_pose:
                    now = rospy.Time.now().to_sec()
                    delta_time_camera =  now - latest_cam_pose.header.stamp.to_sec()
                    delta_time_robot_base = now - latest_robot_base_pose.header.stamp.to_sec()
                    delta_time_marker = now - latest_marker_pose.header.stamp.to_sec()
                    delta_time_ee = now - latest_ee_pose.header.stamp.to_sec()
                    
                    if np.max((delta_time_camera, delta_time_robot_base, delta_time_marker, delta_time_ee)) < time_threshold:
                        A1[:,:,i] = posestamped_to_se3(latest_cam_pose)
                        B1[:,:,i] = posestamped_to_se3(latest_marker_pose, inverse_transformation = True)
                        
                        A2[:,:,i] = posestamped_to_se3(latest_cam_pose, inverse_transformation = True) @ posestamped_to_se3(latest_robot_base_pose)
                        B2[:,:,i] = posestamped_to_se3(latest_ee_pose, inverse_transformation = True)
                        
                        i+=1
                        data_captured = True
                    else:
                        if delta_time_camera > time_threshold:
                            rospy.logerr("No data from camera pose since [s]: " +str(delta_time_camera))
                        if delta_time_robot_base > time_threshold:
                            rospy.logerr("No data from robot base pose since [s]: " +str(delta_time_robot_base))
                        if delta_time_marker > time_threshold:
                            rospy.logerr("No data from marker pose since [s]: " +str(delta_time_marker))
                        if delta_time_ee > time_threshold:
                            rospy.logerr("No data from ee pose since [s]: " +str(delta_time_ee))
                            
                            
                else:
                    if not latest_cam_pose:
                        rospy.logerr("No data received from camera pose topic")
                    if not latest_robot_base_pose:
                        rospy.logerr("No data received from robot pose topic")
                    if not latest_marker_pose:
                        rospy.logerr("No data received from marker pose topic")
                    if not latest_ee_pose:
                        rospy.logerr("No data received from end effector pose topic")
                        
                
                if data_captured:
                    recorded_pub.publish(True)
                    recording = False
                    rospy.loginfo("Picture " +str(i) + "/" + str(n))
                    

            rospy.sleep(1)
            
                    

    except rospy.ROSInterruptException:
        pass
    
    # Determine the path for saving the .npy files
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    # Save the A and B matrices to .npy files in the same directory as the script
    np.save(os.path.join(script_dir, 'A1_matrices.npy'), A1)
    np.save(os.path.join(script_dir, 'B1_matrices.npy'), B1)
    np.save(os.path.join(script_dir, 'A2_matrices.npy'), A2)
    np.save(os.path.join(script_dir, 'B2_matrices.npy'), B2)


if __name__ == '__main__':
    main()
