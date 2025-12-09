#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
import tf
import tf.transformations as tft

#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
import tf
import tf.transformations as tft

class ColorPerceptionNode:
    def __init__(self):
        rospy.init_node("color_perception_node", anonymous=True)

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.tf_listener = tf.TransformListener()
        
        # Parameters loading from ROS server i parametri dal server ROS
        try:
            params = rospy.get_param("/pick_and_place_config")
            p_params = params['perception_params']
            c_ranges = p_params['color_ranges']
        except KeyError as e:
            rospy.logerr(f"Impossibile caricare i parametri di configurazione ROS: {e}")
            rospy.signal_shutdown("Configurazione ROS mancante.")
            return

        self.table_z = p_params['table_height_z']
        self.camera_frame = p_params['camera_frame']
        self.TARGET_FRAME = p_params['base_frame']
        self.color_ranges = {
            "rosso": [c_ranges['rosso_1'], c_ranges['rosso_2']],
            "verde": [c_ranges['verde']], # Nomenclatura corretta
            "blu": [c_ranges['blu']]
        }

        self.pose_pub = rospy.Publisher("/cube_pose_stamped", PoseStamped, queue_size=1)
        self.color_bag_pub = rospy.Publisher("/color_detection/image_debug", Image, queue_size=1)
        self.camera_info_received = False

        rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_callback)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        rospy.loginfo("Nodo Color Perception avviato. Aspettando trasformazioni...")

        try:
            self.tf_listener.waitForTransform(self.TARGET_FRAME, self.camera_frame, rospy.Time(), rospy.Duration(10.0))
            rospy.loginfo("Trasformazioni trovate (Base->Camera).")
        except:
            rospy.logerr("Trasformazioni non disponibili.")
    
    # Callback per ricavare le informazioni della camera
    def camera_callback(self, msg):
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True

    # Ricava il centro (u,v) del cubo in pixel e l'angolo dalla maschera
    def find_cube(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        
        # Filtra i blob (troppo piccoli <500, troppo grandi >10000)
        contours_correct = [c for c in contours if 500 < cv2.contourArea(c) < 10000]
        if not contours_correct:
            return None
        
        # Prendo il contorno più grande
        contour = max(contours_correct, key=cv2.contourArea)
        M = cv2.moments(contour)
        if M["m00"] == 0:
            return None
        
        # Ricavo le coordinate del centro del contorno in pixel
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        (u_center, v_center), (w, h), angle = cv2.minAreaRect(contour)
        
        # Orientamento (utile per Pick-and-Place)
        # Nota: L'angolo di cv2.minAreaRect può variare da -90 a 0 o da 0 a 90 a seconda della libreria
        
        yaw_rad = np.deg2rad(min(abs(angle), abs(90 - angle)))
#        yaw_rad = np.deg2rad(angle) 
        
        return (u, v, yaw_rad)

    # Calcola la posa 3D del cubo dato il centro (u,v) in pixel
    def find_pose(self, u, v, stamp, yaw_rad):
        try:
            # Raggio vettoriale in frame camera
            ray_camera = self.camera_model.projectPixelTo3dRay((u, v))
            ray_camera = np.array(ray_camera)

            # Ricavo la trasformazione da camera a TARGET_FRAME
            self.tf_listener.waitForTransform(self.TARGET_FRAME, self.camera_frame, stamp, rospy.Duration(0.5))
            cam_pos, cam_rot = self.tf_listener.lookupTransform(self.TARGET_FRAME, self.camera_frame, stamp)
            
            # Porto il raggio da camera a TARGET_FRAME
            rot_matrix = tft.quaternion_matrix(cam_rot)[:3, :3]
            ray_world = rot_matrix.dot(ray_camera) 

            # Ricavo il parametro t (forzato al piano del tavolo)
            t = (self.table_z - cam_pos[2]) / ray_world[2]
            if t <= 0:
                return None
            
            # Calcolo la posa 3D
            pose3d = cam_pos + t * ray_world
            
            # Creo il messaggio PoseStamped
            q = tft.quaternion_from_euler(0.0, 0.0, yaw_rad)
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            # frame_id verrà impostato nella image_callback (colore)
            msg.pose.position.x = pose3d[0]
            msg.pose.position.y = pose3d[1]
            msg.pose.position.z = pose3d[2] + 0.01 # Leggero offset Z per il picking
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            return msg

        except Exception as e:
            rospy.logerr(f"Errore nel calcolo della posa 3D: {e}")
            return None

    def image_callback(self, msg):
        if not self.camera_info_received:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Ordine di priorità: rosso > verde > blu
        for colore, ranges in self.color_ranges.items():
            combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            
            # Combina i range per il colore (es. due range per il rosso)
            for r in ranges:
                h_min, s_min, v_min, h_max, s_max, v_max = r
                combined_mask += cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
            
            mask_filtered = cv2.medianBlur(combined_mask, 5)

            result = self.find_cube(mask_filtered)
            
            # Se trova un cubo, calcola la posa e pubblica
            if result:
                u, v, yaw_rad = result
                pose_msg = self.find_pose(u, v, msg.header.stamp, yaw_rad)
                
                if pose_msg is not None:
                    pose_msg.header.frame_id = colore # Imposta l'ID del cubo
                    self.pose_pub.publish(pose_msg)
                    rospy.loginfo(f"Cubo {colore} trovato in x={pose_msg.pose.position.x:.2f}, y={pose_msg.pose.position.y:.2f}, z={pose_msg.pose.position.z:.2f}")
                
                # Debug (Mostra la maschera del cubo attuale)
                cv2.imshow("color_detection", mask_filtered)
                cv2.waitKey(1)
                debug_img_msg = self.bridge.cv2_to_imgmsg(mask_filtered, "mono8")
                debug_img_msg.header = msg.header
                self.color_bag_pub.publish(debug_img_msg)
                return    

        # Se non c'è un cubo colorato rilevato con priorità, mostra l'immagine classica
        cv2.imshow("color_detection", img)
        cv2.waitKey(1)
        debug_img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        debug_img_msg.header = msg.header
        self.color_bag_pub.publish(debug_img_msg)
 

if __name__ == "__main__":
    try:
        ColorPerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
