# Dof4_Robot_Arm

ROS implementation of 4 Dof Robot Arm consisted with 3D printed PLA links and HerkuleX motor Joints(DRS-201/101).

### A. Software prerequisites
1. Ubuntu 16.04+
2. ROS kinetic

### B. Installation
Clone the repository inside src/ directory of your catkin workspace
1. mkdir obstacle_processor
2. cd obstacle_processor/
3. git clone name_of_repository
4. Run CMake to compile source code
    catkin_make
5. roscore
6. rosrun dof4_robot_arm dof4_robot_arm_node
7. rosrun dof4_robot_arm dof4_robot_arm_client

### C. TODO
0. GAZEBO simulation -> Robot moving
1. IOT robot arm control
