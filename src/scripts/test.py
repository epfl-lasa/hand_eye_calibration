#!/usr/bin/env python
import os
import rospy
import transforms3d
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped

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
    Convert PoseStamped message to SE(3) representation (4x4 transformation matrix).
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

def matrix_to_pose_stamped(matrix, frame_id="world"):
    """
    Convert a 4x4 transformation matrix to a PoseStamped message.
    
    Args:
        matrix (numpy.ndarray): A 4x4 transformation matrix.
        frame_id (str): The frame ID in which the Pose is defined.
        
    Returns:
        geometry_msgs.msg.PoseStamped: The PoseStamped message.
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.stamp = rospy.Time.now()
    
    # Extract the translation
    pose_stamped.pose.position.x = matrix[0, 3]
    pose_stamped.pose.position.y = matrix[1, 3]
    pose_stamped.pose.position.z = matrix[2, 3]
    
    # Extract the rotation (quaternion) from the matrix
    quaternion = tf.transformations.quaternion_from_matrix(matrix)
    pose_stamped.pose.orientation.x = quaternion[0]
    pose_stamped.pose.orientation.y = quaternion[1]
    pose_stamped.pose.orientation.z = quaternion[2]
    pose_stamped.pose.orientation.w = quaternion[3]
    
    return pose_stamped

def main():
    # Determine the path for loading the .npy files
    script_dir = os.path.dirname(os.path.realpath(__file__))
    rospy.init_node('calibration_tester')
    
    rospy.Subscriber("/vrpn_client_node/cam_grasp/pose", PoseStamped, cam_callback)
    rospy.Subscriber("aruco_single/pose", PoseStamped, marker_callback)
    pub = rospy.Publisher("/test/pose", PoseStamped, queue_size = 1)
    
    # Load the X and Y matrices from .npy files in the same directory as the script
    X1 = np.load(os.path.join(script_dir, 'X1_matrices.npy'))
    Y1 = np.load(os.path.join(script_dir, 'Y1_matrices.npy'))
    X2 = np.load(os.path.join(script_dir, 'X2_matrices.npy'))
    Y2 = np.load(os.path.join(script_dir, 'Y2_matrices.npy'))
    
    time_threshold = 0.2
    try:
        print("Starting testing process")
        while not rospy.is_shutdown():
            # Store and print the latest data
            if latest_cam_pose and  latest_marker_pose:
                now = rospy.Time.now().to_sec()
                delta_time_camera =  now - latest_cam_pose.header.stamp.to_sec()
                delta_time_marker = now - latest_marker_pose.header.stamp.to_sec()
                
                if np.max((delta_time_camera, delta_time_marker)) < time_threshold:
                    A1 = posestamped_to_se3(latest_cam_pose)
                    B1 = posestamped_to_se3(latest_marker_pose, inverse_transformation = True)
                    error = B1 @ np.linalg.inv(X1) @ np.linalg.inv(A1) @ Y1
                    pub.publish(matrix_to_pose_stamped(error))
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
                    
            rospy.sleep(time_threshold)
            
    except rospy.ROSInterruptException:
        pass
    
    
if __name__ == '__main__':
    main()
