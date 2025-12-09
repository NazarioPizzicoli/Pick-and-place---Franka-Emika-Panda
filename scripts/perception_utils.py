#!/usr/bin/env python3
import rospy
import numpy as np
import tf
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped

# Calcola la posa 3D di un oggetto su un piano (table_z)
# dato il centro (u,v) in pixel e l'orientamento (yaw)
def calculate_3d_pose(u, v, stamp, yaw_rad, camera_model, tf_listener, table_z, target_frame, camera_frame):
    """
    Calcola la posa 3D di un punto (u, v) proiettato sul piano del tavolo (table_z) 
    utilizzando il ray-casting e le trasformazioni ROS (TF).
    """
    try:
        # Raggio vettoriale in frame camera
        ray_camera = camera_model.projectPixelTo3dRay((u, v))
        ray_camera = np.array(ray_camera)

        # Ricavo la trasformazione da camera a TARGET_FRAME
        tf_listener.waitForTransform(target_frame, camera_frame, stamp, rospy.Duration(0.5))
        cam_pos, cam_rot = tf_listener.lookupTransform(target_frame, camera_frame, stamp)
        
        # Porto il raggio da camera a TARGET_FRAME
        rot_matrix = tft.quaternion_matrix(cam_rot)[:3, :3]
        ray_world = rot_matrix.dot(ray_camera) 

        # Ricavo il parametro t (forzato al piano del tavolo)
        # Formula: P_z = C_z + t * R_z => t = (P_z - C_z) / R_z
        t = (table_z - cam_pos[2]) / ray_world[2]
        if t <= 0:
            rospy.logwarn("Punto di impatto non valido (t<=0).")
            return None
        
        # Calcolo la posa 3D
        pose3d = np.array(cam_pos) + t * ray_world
        
        # Creo il messaggio PoseStamped
        q = tft.quaternion_from_euler(0.0, 0.0, yaw_rad)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = target_frame 
        msg.pose.position.x = pose3d[0]
        msg.pose.position.y = pose3d[1]
        msg.pose.position.z = pose3d[2] + 0.01 # Leggero offset Z per il picking
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, ZeroDivisionError) as e:
        rospy.logerr(f"Errore nel calcolo della posa 3D: {e}")
        return None
