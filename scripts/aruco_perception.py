#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Quaternion
from image_geometry import PinholeCameraModel
import tf 
import tf.transformations as tft
from .perception_utils import calculate_3d_pose

class ArucoPerceptionNode:
    def __init__(self):
        rospy.init_node("aruco_perception_node", anonymous=True)

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.tf_listener = tf.TransformListener()
        
        try:
            params = rospy.get_param("/pick_and_place_config")
            p_params = params['perception_params']
        except KeyError as e:
            rospy.logerr(f"Unable to load ROS configuration parameters: {e}")
            rospy.signal_shutdown("Missing ROS configuration")
            return

        self.table_z = p_params['table_height_z']
        self.camera_frame = p_params['camera_frame']
        self.TARGET_FRAME = p_params['base_frame'] 
        
        self.pose_pub = rospy.Publisher("/cube_pose_stamped", PoseStamped, queue_size=1)
        self.aruco_bag_pub = rospy.Publisher("/aruco_detection/image_debug", Image, queue_size=1)
        
        self.camera_info_received = False

        self.MARKER_SIZE = 0.04  # 4cm
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters() 

        rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_callback)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        rospy.loginfo("ArUco Perception node started")
        
        try:
            self.tf_listener.waitForTransform(self.TARGET_FRAME, self.camera_frame, rospy.Time(), rospy.Duration(10.0))
            rospy.loginfo("Transformations found (Base->Camera)")
        except:
            rospy.logerr("Transformations not available")

    def camera_callback(self, msg):
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True

    def image_callback(self, msg):
        if not self.camera_info_received:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is None or len(ids) == 0:
            return
        
        K = np.array(self.camera_model.K).reshape((3,3))
        D = np.array(self.camera_model.D) if self.camera_model.D is not None else np.zeros(5)
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.MARKER_SIZE, K, D)
        rvec = rvecs[0]

        marker_corners = corners[0][0]
        u = int(np.mean(marker_corners[:, 0]))
        v = int(np.mean(marker_corners[:, 1]))

        pose_msg_utility = calculate_3d_pose(
            u, v, msg.header.stamp, 0.0, # Passiamo yaw=0.0 perché l'orientamento viene calcolato separatamente con rvec
            self.camera_model, self.tf_listener, 
            self.table_z, self.TARGET_FRAME, self.camera_frame
        )
        
        if pose_msg_utility is None:
            rospy.logwarn("Invalid impact point (t<=0) or TF error")
            return

        try:
            self.tf_listener.waitForTransform(self.TARGET_FRAME, self.camera_frame, msg.header.stamp, rospy.Duration(0.5))
            _, cam_rot = self.tf_listener.lookupTransform(self.TARGET_FRAME, self.camera_frame, msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Error in calculating orientation with TF: {e}")
            return
            
        R, _ = cv2.Rodrigues(rvec)
        matrix = np.eye(4)
        matrix[:3, :3] = R
        q_marker_in_cam = tft.quaternion_from_matrix(matrix)
        
        q_cam_in_base = cam_rot 

        q_marker_in_base = tft.quaternion_multiply(q_cam_in_base, q_marker_in_cam)

        (roll, pitch, yaw) = tft.euler_from_quaternion(q_marker_in_base)
        q_pure_yaw = tft.quaternion_from_euler(0.0, 0.0, yaw)
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "aruco" 
        pose_msg.pose.position = pose_msg_utility.pose.position
        pose_msg.pose.orientation = Quaternion(*q_pure_yaw)

        self.pose_pub.publish(pose_msg)
        rospy.loginfo_throttle(1.0, f"Aruco cube found in x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
            
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        tvec = tvecs[0] 
        cv2.drawFrameAxes(img, K, D, rvec, tvec[0], 0.05) 
        
        debug_img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        debug_img_msg.header = pose_msg.header
        self.aruco_bag_pub.publish(debug_img_msg)

if __name__=="__main__":
    node = ArucoPerceptionNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
