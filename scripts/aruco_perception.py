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

class ArucoPerceptionNode:
    def __init__(self):
        rospy.init_node("aruco_perception_node", anonymous=True)

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.tf_listener = tf.TransformListener()
        
        # Carica i parametri dal server ROS
        try:
            params = rospy.get_param("/pick_and_place_config")
            p_params = params['perception_params']
        except KeyError as e:
            rospy.logerr(f"Impossibile caricare i parametri di configurazione ROS: {e}")
            rospy.signal_shutdown("Configurazione ROS mancante.")
            return

        self.table_z = p_params['table_height_z']
        self.camera_frame = p_params['camera_frame']
        self.TARGET_FRAME = p_params['base_frame'] 
        
        self.pose_pub = rospy.Publisher("/cube_pose_stamped", PoseStamped, queue_size=1)
        self.aruco_bag_pub = rospy.Publisher("/aruco_detection/image_debug", Image, queue_size=1)
        
        self.camera_info_received = False

        self.MARKER_SIZE = 0.04  # 4cm
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Gestione compatibilità OpenCV per DetectorParameters
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters() 

        rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_callback)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        rospy.loginfo("Nodo ArUco Perception avviato")
        
        try:
            self.tf_listener.waitForTransform(self.TARGET_FRAME, self.camera_frame, rospy.Time(), rospy.Duration(10.0))
            rospy.loginfo("Trasformazioni trovate (Base->Camera).")
        except:
            rospy.logerr("Trasformazioni non disponibili.")

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
        
        # Processa solo il primo ID trovato
        if ids is None or len(ids) == 0:
            return
        
        # Estrai matrici della camera
        K = np.array(self.camera_model.K).reshape((3,3))
        D = np.array(self.camera_model.D) if self.camera_model.D is not None else np.zeros(5)
        
        # Stima la posa per il primo marker trovato
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.MARKER_SIZE, K, D)
        tvec = tvecs[0]
        rvec = rvecs[0]

        # 1. Calcoliamo il centro del marker in pixel (u,v)
        marker_corners = corners[0][0]
        u = int(np.mean(marker_corners[:, 0]))
        v = int(np.mean(marker_corners[:, 1]))

        # 2. Proiettiamo un raggio da (u,v) per trovare la posizione 3D sul tavolo
        try:
            ray_camera = self.camera_model.projectPixelTo3dRay((u, v))
            ray_camera = np.array(ray_camera)
            
            # Usiamo il timestamp del messaggio (msg.header.stamp) per la TF
            self.tf_listener.waitForTransform(self.TARGET_FRAME, self.camera_frame, msg.header.stamp, rospy.Duration(0.5))
            cam_pos, cam_rot = self.tf_listener.lookupTransform(self.TARGET_FRAME, self.camera_frame, msg.header.stamp) # Fix

            rot_matrix = tft.quaternion_matrix(cam_rot)[:3, :3]
            ray_target_frame = rot_matrix.dot(ray_camera)

            t = (self.table_z - cam_pos[2]) / ray_target_frame[2] # Usa self.table_z
            if t <= 0:
                rospy.logwarn("Punto di impatto non valido (t<=0).")
                return
                
            hit_point = np.array(cam_pos) + t * ray_target_frame

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, ZeroDivisionError) as e:
            rospy.logwarn(f"Errore nel calcolo della posa con ray-casting: {e}")
            return
        
        # 3. Creiamo il messaggio di posa finale
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "aruco" 
        pose_msg.pose.position.x = hit_point[0]
        pose_msg.pose.position.y = hit_point[1]
        pose_msg.pose.position.z = hit_point[2] + 0.01 # Offset Z per il picking
        
        R, _ = cv2.Rodrigues(rvec)
        matrix = np.eye(4)
        matrix[:3, :3] = R
        q_marker_in_cam = tft.quaternion_from_matrix(matrix)
        
        q_cam_in_base = cam_rot 

        # Orientamento del marker nel frame 'panda_link0'
        q_marker_in_base = tft.quaternion_multiply(q_cam_in_base, q_marker_in_cam)

        # Pubblichiamo solo lo yaw puro (Roll=0, Pitch=0)
        (roll, pitch, yaw) = tft.euler_from_quaternion(q_marker_in_base)
        q_pure_yaw = tft.quaternion_from_euler(0.0, 0.0, yaw)
        
        pose_msg.pose.orientation = Quaternion(*q_pure_yaw)

        self.pose_pub.publish(pose_msg)
        rospy.loginfo_throttle(1.0, f"Cubo aruco trovato in x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
            
        # Visualizzazione per debug
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
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