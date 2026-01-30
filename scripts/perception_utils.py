#!/usr/bin/env python3
import rospy
import numpy as np
import tf
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped


def calculate_3d_pose(u, v, stamp, yaw_rad, camera_model, tf_listener, table_z, target_frame, camera_frame):
    """
    Calculate the 3D position of a point (u, v) projected onto the table plane (table_z) 
    using ray casting and ROS transformations (TF).
    """
    try:
        ray_camera = camera_model.projectPixelTo3dRay((u, v))
        ray_camera = np.array(ray_camera)

        tf_listener.waitForTransform(target_frame, camera_frame, stamp, rospy.Duration(0.5))
        cam_pos, cam_rot = tf_listener.lookupTransform(target_frame, camera_frame, stamp)
        
        rot_matrix = tft.quaternion_matrix(cam_rot)[:3, :3]
        ray_world = rot_matrix.dot(ray_camera) 

        # Formula: P_z = C_z + t * R_z => t = (P_z - C_z) / R_z
        t = (table_z - cam_pos[2]) / ray_world[2]
        if t <= 0:
            rospy.logwarn("Invalid point of impact (t<=0).")
            return None
        
        pose3d = np.array(cam_pos) + t * ray_world
        
        q = tft.quaternion_from_euler(0.0, 0.0, yaw_rad)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = target_frame 
        msg.pose.position.x = pose3d[0]
        msg.pose.position.y = pose3d[1]
        msg.pose.position.z = pose3d[2] + 0.01
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, ZeroDivisionError) as e:
        rospy.logerr(f"Error in 3D layout calculation: {e}")
        return None
