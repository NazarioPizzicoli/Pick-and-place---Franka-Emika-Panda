cmake_minimum_required(VERSION 3.0.2)
project(franka_lab_dev)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs # Aggiunto per chiarezza, anche se già in package.xml
  sensor_msgs   # Aggiunto per chiarezza, anche se già in package.xml
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py.
## This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## (Sezione per Messaggi/Servizi non utilizzati, lasciata commentata)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## (Sezione per Dynamic Reconfigure non utilizzato, lasciata commentata)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES franka_lab_dev
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## (Sezioni per librerie ed eseguibili C++ lasciate commentate)


#############
## Install ##
#############

# Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/pick_and_place_core.py
  scripts/color_perception.py
  scripts/aruco_perception.py
  scripts/perception_utils.py
  scripts/spawn_red_cube.py
  scripts/spawn_green_cube.py
  scripts/spawn_blue_cube.py
  scripts/spawn_aruco_cube.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)

# Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## (Sezioni per il Testing lasciate commentate)
