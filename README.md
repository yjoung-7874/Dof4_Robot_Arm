# Dof4_Robot_Arm

Raspberry pi + ROS implementation of 4 Dof Robot Arm consisted with 3D printed PLA links and HerkuleX motor Joints(DRS-201/101).

### A. Software prerequisites
1. Ubuntu 16.04+
2. ROS kinetic

### B. Installation
Clone the repository inside src/ directory of your catkin workspace
1. git clone "link of repository"
2. cd ~/'your catkin workspace' && catkin_make
3. roscore
4. rosrun dof4_robot_arm dof4_robot_arm_node
5. rosrun dof4_robot_arm dof4_robot_arm_client

### C. TODO
0. GAZEBO simulation -> Robot moving
1. IOT robot arm control
