#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def main():
    rospy.init_node("spawn_aruco_cube")

    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    model_path = rospy.get_param("~model_path", 
        "/home/naza/franka_ws/src/laboratorio/urdf/aruco_cube.urdf")

    with open(model_path, "r") as f:
        model_xml = f.read()

    pose = Pose()
    pose.position.x = 0.45
    pose.position.y = 0.005
    pose.position.z = 1.031998
    pose.orientation.w = 1.0

    try:
        resp = spawn_model("aruco_cube", model_xml, "/", pose, "world")
        rospy.loginfo("Cubo ArUco spawnato ✅")
    except Exception as e:
        rospy.logerr("Errore spawn: %s", e)

if __name__ == "__main__":
    main()
