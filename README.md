# Dof4_Robot_Arm

Raspberry pi + ROS implementation of 4 Dof Robot Arm consisted with 3D printed PLA links and HerkuleX motor Joints(DRS-201/101).

### A. Software prerequisites
1. Ubuntu 16.04+
2. ROS kinetic

### B. System Diagram
![alt text](https://github.com/yjoung-7874/Dof4_Robot_Arm/blob/master/diagram.png)

### C. Installation
Clone the repository inside src/ directory of your catkin workspace
```
git clone "link of repository"
cd ~/'your catkin workspace' && catkin_make
roscore
rosrun dof4_robot_arm dof4_robot_arm_node
rosrun dof4_robot_arm dof4_robot_arm_client
```
### D. Experiment Video
https://youtu.be/I8n40YT5L0M
https://www.youtube.com/shorts/I8n40YT5L0M

### E. TODO
0. GAZEBO simulation -> Robot moving
1. IOT robot arm control
