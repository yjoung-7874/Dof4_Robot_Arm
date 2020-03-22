#include "ros/ros.h"
#include <center_bridge/MotorValue.h>
#include "HerkuleX.h"
#include <vector>
#include <iostream>

using namespace std;

bool value_assign(center_bridge::MotorValue::Request &req,
        center_bridge::MotorValue::Response &res) {

	HerkuleX motor;
	motor.TorqueOn(BROADCAST_ID);
	cout << "Torque on..." << endl;
	map<int, int> id_motor_values = {
					 {0,0}, {1,0}, {2,0}, {3,0}, {4,0},
					 {5,0}, {6,0}, {7,0}, {8,0}, {9,0}
					};

	// job description 0 : torque on
	// job description 1 : single motor operation
	// job description 2 : multiple motor operation
	// job description 3 : torque off
	int i = 0;
	for(i = 0; i < req.job_description.size(); i++){
		if (req.job_description.at(i) == 0) {
			cout << "Torque On ID = " << req.id.at(i) << endl;
			motor.TorqueOn(req.id.at(i));
		} else if (req.job_description.at(i) == 3) {
			cout << "Torque On ID = " << req.id.at(i) << endl;
			motor.TorqueOff(req.id.at(i));
		} else if (req.job_description.at(i) == 2) {
			// build map for multiple motors
                        id_motor_values.at(req.id.at(i)) = req.motor_value.at(i);
			motor.movePos(id_motor_values, req.playtime.at(i), req.led.at(i));
                        ROS_INFO("saved: job_description=%ld, id=%ld, motor_value = %ld, playtime = %ld, led = %ld",
                                (long int)req.job_description.at(i), (long int)req.id.at(i), (long int)req.motor_value.at(i),
                                (long int)req.playtime.at(i), (long int)req.led.at(i));
				//motor.movePos(id_motor_values, req.playtime.at(i), req.led.at(i));
		} else if (req.job_description.at(i) == 1) { // single motor operation
			motor.movePos(req.id.at(i), req.motor_value.at(i), req.playtime.at(i), req.led.at(i));

                        ROS_INFO("request: job_description=%ld, id=%ld, motor_value = %ld, playtime = %ld, led = %ld",
                                (long int)req.job_description.at(i), (long int)req.id.at(i), (long int)req.motor_value.at(i),
                                (long int)req.playtime.at(i), (long int)req.led.at(i));
                        ROS_INFO("get angle from motor id : [%ld], angle: [%lf]", (long int)req.id.at(i), motor.getAngle(req.id.at(i)));
                } /*else {
                        if (id_motor_values.size() != 0) { // multiple motors operation
                                motor.movePos(id_motor_values, req.playtime.at(i), req.led.at(i));
                                ROS_INFO("multiple motor angle assigned");
                        }
		}*/
	}
/*
	if (id_motor_values.size() != 0){
		motor.movePos(id_motor_values, req.playtime.at(i), req.led.at(i));
		//id_motor_values.clear();
		ROS_INFO("check");
                ROS_INFO("multiple motor angle assigned");
	}
*/
//        motor.TorqueOff(BROADCAST_ID);
//        cout << "Torque off..." << endl;

	res.result = 1;

        return true;
}
//id, speed, playtime, led,pos, motor_values(map), angle
// req
// vector arr[] for job description
// map<int, int> for motor id and motor_values


int main(int argc, char **argv)
{
        ros::init(argc, argv, "motor_driver");
        ros::NodeHandle n;

        ros::ServiceServer service = n.advertiseService("Motor_Value", value_assign);
        ros::spin();

        return 0;
 }

