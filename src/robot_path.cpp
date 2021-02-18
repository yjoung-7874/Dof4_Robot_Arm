#include "ros/ros.h"
// #include <iostream>
#include <cstdlib>
#include <dof4_robot_arm/Value2Robot.h>
#include <vector>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dof4_robot_arm::Value2Robot>("Cmd2Dof4Robot");
    dof4_robot_arm::Value2Robot srv;
    
    vector<vector<float> > desired_arr ({
        vector<float>( { 50,  0,   80,  90, 0, 1 }),
        vector<float>( { 100, 0,   80,  90, 0, 0.5 }),
        vector<float>( { 100, 0,   120, 90, 0, 0.5 }),
        vector<float>( { 50,  0,   120, 90, 0, 0.5 }),
        vector<float>( { 0,   50,  120, 90, 0, 1 }),
	vector<float>( { 0,   100, 120, 90, 0, 0.5 }),
        vector<float>( { 0,   100, 80,  90, 0, 0.5 }),
	vector<float>( { 0,   50,  80, 90,  0, 0.5 }),

    });


    while(ros::ok()){
        for (int i = 0; i < desired_arr.size(); i++){
            srv.request.x = desired_arr[i][0];
            srv.request.y = desired_arr[i][1];
            srv.request.z = desired_arr[i][2];
            srv.request.alp = desired_arr[i][3];
            srv.request.t0 = desired_arr[i][4];
	    srv.request.tf = desired_arr[i][5];
            if (i == 0) cout << "moving in to oven" << endl;
            if (i == 4) cout << "putting in to storage" << endl;

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
    }

  return 0;
}
