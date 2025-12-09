#!/usr/bin/env python3
import rospy
import sys
import copy
import moveit_commander
import geometry_msgs.msg
import numpy as np
import tf.transformations as tft

class PickAndPlaceCore:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_and_place_core_node", anonymous=True) 

        # Load parameters
        try:
            self.params = rospy.get_param("/pick_and_place_config")
            p_params = self.params['pick_parameters']
            r_settings = self.params['robot_settings']
            self.joint_targets = self.params['joint_targets']
        except KeyError as e:
            rospy.logerr(f"Impossible to load ROS configuration parameters: {e}. Check pick_and_place_params.yaml.")
            sys.exit(1)

        # Parametri
        self.ROLL_FIXED = p_params['roll_fixed']
        self.PITCH_FIXED = p_params['pitch_fixed']
        self.YAW_COMPENSATION = p_params['yaw_compensation']
        self.PRE_PICK_OFFSET_Z = p_params['pre_pick_offset_z']
        self.PICK_OFFSET_Z = p_params['pick_offset_z']
        self.home_joints = self.joint_targets['home']

        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm") 
        self.hand_group = moveit_commander.MoveGroupCommander("panda_hand") 
        self.arm_group.set_planning_time(r_settings['planning_time'])
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_max_velocity_scaling_factor(r_settings['velocity_scaling_factor'])
        self.arm_group.set_max_acceleration_scaling_factor(r_settings['velocity_scaling_factor'])
        
        # Stato della State Machine: Flag di blocco
        self.is_busy = False 
        self.target_sub = None # Variabile per tenere traccia dell'oggetto Subscriber
        self.start_subscription() # NUOVO: Avvia la sottoscrizione
        rospy.loginfo("Pick and Place Core avviato. In attesa del target...")
        # Inizializzazione a Home (Movimento bloccante)
        rospy.sleep(1.0) # Aspetta che moveit sia pronto
        self.move_to_joints(self.home_joints, "Home Iniziale")
        
        # Sottoscrizione al topic di Percezione
        # Queue size 1 assicura che il subscriber conservi solo l'ultimo messaggio ricevuto.
        #rospy.Subscriber("/cube_pose_stamped", geometry_msgs.msg.PoseStamped, self.target_callback, queue_size=1) 
        #rospy.loginfo("Pick and Place Core avviato. In attesa del target...")
        # Inizializzazione a Home (Movimento bloccante)
        #rospy.sleep(1.0) # Aspetta che moveit sia pronto
        #self.move_to_joints(self.home_joints, "Home Iniziale")
    # NUOVI METODI PER GESTIRE LA SOTTOSCRIZIONE
    def start_subscription(self):
        """Attiva la sottoscrizione al topic /cube_pose_stamped."""
        if self.target_sub is None:
            self.target_sub = rospy.Subscriber("/cube_pose_stamped", geometry_msgs.msg.PoseStamped, self.target_callback, queue_size=1)
            rospy.loginfo("Sottoscrizione topic attivata.")

    def stop_subscription(self):
        """Disattiva la sottoscrizione, svuotando di fatto il buffer."""
        if self.target_sub is not None:
            self.target_sub.unregister()
            self.target_sub = None
            rospy.loginfo("Sottoscrizione topic disattivata e buffer svuotato.")
    # --- Controller Methods (Invariati) ---
    def open_gripper(self):
        rospy.loginfo("Apro il gripper...")
        self.hand_group.set_joint_value_target([0.04, 0.04])
        self.hand_group.go(wait=True)
        self.hand_group.stop()
        rospy.sleep(0.5)
    
    def close_gripper(self):
        rospy.loginfo("Chiudo il gripper...")
        self.hand_group.set_joint_value_target([0.00, 0.00])
        self.hand_group.go(wait=True)
        self.hand_group.stop()
        rospy.sleep(0.5)

    def move_to_pose(self, poseStamped, name=""):
        rospy.loginfo(f"Mi sposto verso la posa: {name}")
        self.arm_group.set_pose_target(poseStamped)
        
        success, plan, _, error_code = self.arm_group.plan()
        
        if success:
            self.arm_group.execute(plan, wait=True)
            rospy.loginfo(f"Movimento a {name} riuscito.")
            result = True
        else:
            rospy.logerr(f"Pianificazione fallita per la posa: {name}. Codice errore: {error_code.val}")
            result = False

        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        rospy.sleep(0.5)
        return result

    def move_to_joints(self, joint_value, name=""):
        rospy.loginfo(f"Mi sposto verso la configurazione di giunti: {name}")
        self.arm_group.set_joint_value_target(joint_value)
        
        success = self.arm_group.go(wait=True)
        
        if success:
            rospy.loginfo(f"Movimento a {name} riuscito.")
            result = True
        else:
            rospy.logerr(f"Movimento a giunti {name} fallito.")
            result = False

        self.arm_group.stop()
        rospy.sleep(0.5)
        return result

    # --- State Machine Logic ---

    def target_callback(self, msg):
        """Gestisce il target ricevuto."""
        
        # 1. Filtro BUSY: Se il robot si sta già muovendo, ignora il messaggio in arrivo
        if self.is_busy:
            return

        # 2. Target Acquisito: Blocca il sistema e avvia la sequenza
        self.is_busy = True 
        self.stop_subscription()
        rospy.loginfo(f"\n--- TARGET '{msg.header.frame_id}' ACQUISITO ---")
        
        self.execute_sequence(msg)
        
        # 3. PAUSA DI RESET (La tua idea del "reset/flush")
        # Aspetta che il nodo di percezione abbia il tempo di notare la rimozione del cubo
        rospy.loginfo("--- PAUSA DI PULIZIA SCENA (3 sec) ---")
        rospy.sleep(3.0) 
        self.start_subscription()
        self.is_busy = False # Sblocca: pronto per il prossimo target
        rospy.loginfo("--- SEQUENZA COMPLETATA. IN ATTESA DI NUOVO TARGET ---")
            
    def execute_sequence(self, target_pose):
        """Esegue la sequenza completa Pick-and-Place."""
        
        cube_id = target_pose.header.frame_id 

        # Calcolo Orientamento Compensato (omesso per brevità, stessa logica)
        q_input = [
            target_pose.pose.orientation.x, target_pose.pose.orientation.y,
            target_pose.pose.orientation.z, target_pose.pose.orientation.w
        ]
        _, _, y_cube = tft.euler_from_quaternion(q_input)
        y_finale = y_cube + self.YAW_COMPENSATION
        q_finale = tft.quaternion_from_euler(self.ROLL_FIXED, self.PITCH_FIXED, y_finale)

        # Preparazione delle Pose di Pick
        pre_pick_pose = geometry_msgs.msg.PoseStamped() 
        pre_pick_pose.header.frame_id = "panda_link0"
        pre_pick_pose.pose.position.x = target_pose.pose.position.x
        pre_pick_pose.pose.position.y = target_pose.pose.position.y
        pre_pick_pose.pose.position.z = target_pose.pose.position.z + self.PRE_PICK_OFFSET_Z
        pre_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*q_finale) 
        
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = "panda_link0"
        pick_pose.pose.position.x = target_pose.pose.position.x
        pick_pose.pose.position.y = target_pose.pose.position.y
        pick_pose.pose.position.z = target_pose.pose.position.z + self.PICK_OFFSET_Z 
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*q_finale) 
        
        post_pick_pose = copy.deepcopy(pre_pick_pose)
        
        # Recupera le pose Place da YAML
        try:
            place_joints = self.joint_targets[f'place_{cube_id}']
            pre_place_joints = self.joint_targets[f'pre_place_{cube_id}']
        except KeyError:
            rospy.logerr(f"Pose Place non definite in YAML per l'ID: {cube_id}. Fallback a Home.")
            self.move_to_joints(self.home_joints, "Home Fallback")
            return

        # PIPELINE PICK & PLACE
        
        self.move_to_joints(self.home_joints, "Home")
        self.open_gripper()
        
        # Movimenti Pick (Se uno fallisce, la funzione interna ritorna False e la sequenza si ferma)
        if not self.move_to_pose(pre_pick_pose, "Pre-Pick"): return
        if not self.move_to_pose(pick_pose, "Pick"): return
        
        self.close_gripper()
        
        # Movimenti Place
        if not self.move_to_pose(post_pick_pose, "Post-Pick"): return
        if not self.move_to_joints(pre_place_joints, "Pre-Place"): return
        if not self.move_to_joints(place_joints, "Place"): return
        
        self.open_gripper()
        
        self.move_to_joints(self.home_joints, "Home Finale")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = PickAndPlaceCore()
        node.run()
    except rospy.ROSInterruptException:
        pass