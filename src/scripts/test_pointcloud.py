#!/usr/bin/env python
import os
import rospy
import ros_numpy
import transforms3d
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import open3d as o3d
from std_msgs.msg import Header

# Global variables to store the latest data
latest_ee_pose = None
latest_pc_pose = None

# Callback for the marker pose
def ee_pose_callback(msg):
    global latest_ee_pose
    latest_ee_pose = msg

# Callback for the marker pose
def pointcloud_callback(msg):
    global latest_pc_pose
    latest_pc_pose = msg

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

def pointcloud_to_se3(pointcloud, inverse_transformation=False):
    """
    Converts a PointCloud2 object to a set of SE(3) 4x4 transformation matrices.
    
    Args:
        pointcloud (sensor_msgs.msg.PointCloud2): The input point cloud.
        
    Returns:
        np.ndarray: A 4x4xn array of SE(3) transformation matrices.
    """
    # Initialize the transformation matrix set
    points_list = list(pc2.read_points(pointcloud, skip_nans=True, field_names=("x", "y", "z")))
    
    n_points = len(points_list)
    se3_matrices = np.zeros((4, 4, n_points))
    
    # Populate the matrices
    if inverse_transformation:
        for i, (x, y, z) in enumerate(points_list):
            se3_matrices[:, :, i] = np.array([
                [1, 0, 0, -x],
                [0, 1, 0, -y],
                [0, 0, 1, -z],
                [0, 0, 0, 1]
            ])
    else:
        for i, (x, y, z) in enumerate(points_list):
            se3_matrices[:, :, i] = np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])
    
    return se3_matrices

def pointcloud2_to_open3d(ros_pc2):
    points = []
    for point in pc2.read_points(ros_pc2, field_names=("x", "y", "z"), skip_nans=True):
        points.append([point[0], point[1], point[2]])
    open3d_pc = o3d.geometry.PointCloud()
    open3d_pc.points = o3d.utility.Vector3dVector(np.array(points))
    return open3d_pc

def open3d_to_pointcloud2(open3d_pc, frame_id="camera_color_optical_frame"):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    points = np.asarray(open3d_pc.points)
    ros_pc2 = pc2.create_cloud_xyz32(header, points)
    return ros_pc2



def main():
    # Determine the path for loading the .npy files
    script_dir = os.path.dirname(os.path.realpath(__file__))
    rospy.init_node('calibration_tester')

    rospy.Subscriber("calibration/ee_pose", PoseStamped, ee_pose_callback)
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, pointcloud_callback)
    pub = rospy.Publisher("new_pc", PointCloud2, queue_size = 1)
    
    # Load the X and Y matrices from .npy files in the same directory as the script
    X1 = np.load(os.path.join(script_dir, 'X1_matrices.npy'))
    
    time_threshold = 0.2
    try:
        print("Starting testing process")
        while not rospy.is_shutdown():
            # Store and print the latest data
            if latest_ee_pose and  latest_pc_pose:
                now = rospy.Time.now().to_sec()
                delta_time_ee =  now - latest_ee_pose.header.stamp.to_sec()
                delta_time_pc = now - latest_pc_pose.header.stamp.to_sec()
                
                if np.max((delta_time_ee, delta_time_pc)) < time_threshold:
                    A1 = posestamped_to_se3(latest_ee_pose)

                    B1 = pointcloud2_to_open3d(latest_pc_pose)
                    new_pc = B1.transform(X1).transform(A1)
                    pub.publish(open3d_to_pointcloud2(new_pc))
                    print("published")
                else:
                    if delta_time_ee > time_threshold:
                        rospy.logerr("No data from ee pose since [s]: " +str(delta_time_ee))
                    if delta_time_pc > time_threshold:
                        rospy.logerr("No data from pc pose since [s]: " +str(delta_time_pc))
                        
                        
            else:
                if not latest_ee_pose:
                    rospy.logerr("No data received from ee pose topic")
                if not latest_pc_pose:
                    rospy.logerr("No data received from pc pose topic")
                    
            rospy.sleep(time_threshold)
            
    except rospy.ROSInterruptException:
        pass
    
    
if __name__ == '__main__':
    main()
