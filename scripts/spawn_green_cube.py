#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
import os

def spawn_cube():
    rospy.init_node("spawn_green_cube", anonymous=True)

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_model_prox = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    cube_sdf = """
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <model name="green_cube">
        <static>false</static>
        <link name="link">
          <inertial>
            <mass>0.1</mass>
            <inertia>
              <ixx>0.0001</ixx>
              <iyy>0.0001</iyy>
              <izz>0.0001</izz>
            </inertia>
          </inertial>
          <visual name="visual">
            <geometry>
              <box>
                <size>0.05 0.05 0.05</size>
              </box>
            </geometry>
            <material>
              <ambient>0 1 0 1</ambient>
              <diffuse>0 1 0 1</diffuse>
            </material>
          </visual>
          <collision name="collision">
            <geometry>
              <box>
                <size>0.05 0.05 0.05</size>
              </box>
            </geometry>
            <surface>
              <friction>
                <ode>
                  <mu>1.0</mu>
                  <mu2>1.0</mu2>
                </ode>
              </friction>
            </surface>
          </collision>
        </link>
      </model>
    </sdf>
    """

    cube_pose = Pose()
    cube_pose.position.x = 0.30
    cube_pose.position.y = -0.20
    cube_pose.position.z = 1.04
    cube_pose.orientation.x = 0.0
    cube_pose.orientation.y = 0.0
    cube_pose.orientation.z = 3.0
    cube_pose.orientation.w = 1.0

    try:
        resp = spawn_model_prox(model_name="green_cube",
                               model_xml=cube_sdf,
                               robot_namespace="/",
                               initial_pose=cube_pose,
                               reference_frame="world")
        rospy.loginfo("Cubetto rosso spawnato in Gazebo ✅")
    except rospy.ServiceException as e:
        rospy.logerr("Errore spawn cubo: %s", str(e))

if __name__ == "__main__":
    spawn_cube()
