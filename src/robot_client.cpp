#include "ros/ros.h"
// #include <iostream>
#include <cstdlib>
#include <dof4_robot_arm/Value2Robot.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dof4_robot_arm::Value2Robot>("Cmd2Dof4Robot");
    dof4_robot_arm::Value2Robot srv;

    float desired[6]= {0, 0, 0, 0, 0, 0};

    while(ros::ok()){
        cout << "desired x :" << endl;
        cin >> desired[0];
        cout << "desired y :" << endl;
        cin >> desired[1];
        cout << "desired z :" << endl;
        cin >> desired[2];
        cout << "desired alpha :" << endl;
        cin >> desired[3];
        cout << "desired t0 :" << endl;
        cin >> desired[4];
        cout << "desired tf :" << endl;
        cin >> desired[5];


        srv.request.x = desired[0];
        srv.request.y = desired[1];
        srv.request.z = desired[2];
        srv.request.alp = desired[3];
        srv.request.t0 = desired[4];
        srv.request.tf = desired[5];

        if (client.call(srv)) {
            ROS_INFO("Return Status: %ld", (long int)srv.response.result);
            ROS_INFO("Current x    : %f", srv.response.cur_x);
            ROS_INFO("Current y    : %f", srv.response.cur_y);
            ROS_INFO("Current z    : %f", srv.response.cur_z);
            ROS_INFO("Current alpha: %f", srv.response.cur_alp);
        } else {
            ROS_ERROR("Failed to call service Cmd2Dof4Robot");
            return 1;
        }
    }

  return 0;
}
