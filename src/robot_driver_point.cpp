// Control 4 dof robot arm

// Path Planning (x =  a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0
/* x      = a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0
 * y      = b5*t^5 + b4*t^4 + b3*t^3 + b2*t^2 + b1*t + b0
 * z      = c5*t^5 + c4*t^4 + c3*t^3 + c2*t^2 + c1*t + c0
 * alp    = d5*t^5 + d4*t^4 + d3*t^3 + d2*t^2 + d1*t + d0
 
 * d_x    = 5*a5*t^4 + 4*a4*t^3 + 3*a3*t^2 + 2*a2*t + a1
 * d_y    = 5*b5*t^4 + 4*b4*t^3 + 3*b3*t^2 + 2*b2*t + b1
 * d_z    = 5*c5*t^4 + 4*c4*t^3 + 3*c3*t^2 + 2*c2*t + c1
 * d_alp  = 5*d5*t^4 + 4*d4*t^3 + 3*d3*t^2 + 2*d2*t + d1
 
 * dd_x   = 20*a5*t^3 + 12*a4*t^2 + 6*a3*t +2*a2
 * dd_y   = 20*b5*t^3 + 12*b4*t^2 + 6*b3*t +2*b2
 * dd_z   = 20*c5*t^3 + 12*c4*t^2 + 6*c3*t +2*c2
 * dd_alp = 20*d5*t^3 + 12*d4*t^2 + 6*d3*t +2*d2
 */

/* initial                  matrix                         coefficient
 * & final                  of                             of
 * point                    time                           equations
 * (6x1)     =              (6x6)                       *  (6x1)
 ---------------------------------------------------------------------
 * |x0   |     |t0^5    t0^4    t0^3    t0^2   t0   1|     |a5| 
 * |d_x0 |     |5*t0^4  4*t0^3  3*t0^2  2*t0   1    0|     |a4|
 * |dd_x0|     |20*t0^3 12*t0^2 6*t0    1      0    0|     |a3|
 * |xf   |   = |tf^5    tf^4    tf^3    tf^2   tf   1|  *  |a2|
 * |d_xf |     |5*tf^4  4*tf^3  3*tf^2  2*tf   1    0|     |a1|
 * |dd_xf|     |20*tf^3 12*tf^2 6*tf    1      0    0|     |a0|
  
 * |y0   |     |t0^5    t0^4    t0^3    t0^2   t0   1|     |b5|
 * |d_y0 |     |5*t0^4  4*t0^3  3*t0^2  2*t0   1    0|     |b4|
 * |dd_y0|     |20*t0^3 12*t0^2 6*t0    1      0    0|     |b3|
 * |yf   |   = |tf^5    tf^4    tf^3    tf^2   tf   1|  *  |b2|
 * |d_yf |     |5*tf^4  4*tf^3  3*tf^2  2*tf   1    0|     |b1|
 * |dd_yf|     |20*tf^3 12*tf^2 6*tf    1      0    0|     |b0|
  
 * |z0   |     |t0^5    t0^4    t0^3    t0^2   t0   1|     |c5|
 * |d_z0 |     |5*t0^4  4*t0^3  3*t0^2  2*t0   1    0|     |c4|
 * |dd_z0|     |20*t0^3 12*t0^2 6*t0    1      0    0|     |c3|
 * |zf   |   = |tf^5    tf^4    tf^3    tf^2   tf   1|  *  |c2|
 * |d_zf |     |5*tf^4  4*tf^3  3*tf^2  2*tf   1    0|     |c1|
 * |dd_zf|     |20*tf^3 12*tf^2 6*tf    1      0    0|     |c0|
 
 * |alp0   |   |t0^5    t0^4    t0^3    t0^2   t0   1|     |c5|
 * |d_alp0 |   |5*t0^4  4*t0^3  3*t0^2  2*t0   1    0|     |c4|
 * |dd_alp0|   |20*t0^3 12*t0^2 6*t0    1      0    0|     |c3|
 * |alpf   | = |tf^5    tf^4    tf^3    tf^2   tf   1|  *  |c2|
 * |d_alpf |   |5*tf^4  4*tf^3  3*tf^2  2*tf   1    0|     |c1|
 * |dd_alpf|   |20*tf^3 12*tf^2 6*tf    1      0    0|     |c0|
 */

// Jacobian J from Forward Kinematics d_p = J * d_q
/* J = | -sin(q0) * (l1 * sin(q1) + l2 * sin(q1+q2) + l3 * sin(q1+q2+q3))  | cos(q0) * (l1 * cos(q1) + l2 * cos(q1+q2) + l3 * cos(q1+q2+q3))
 *     | cos(q0) * (l1 * sin(q1) + l2 * sin(q1+q2) + l3 * sin(q1+q2+q3))   | sin(q0) * (l1 * cos(q1) + l2 * cos(q1+q2) + l3 * cos(q1+q2+q3))
 *     | 0                                                                 | -(l1 * sin(q1) + l2 * sin(q1+q2) + l3 * sin(q1+q2+q3))
 *     | 0                                                                 | 1
    // | 1                                                                 | 0                                                              // beta
 
 *     | cos(q0) * (l2 * cos(q1+q2) + l3 * cos(q1+q2+q3))                  | l3 * cos(q0) * cos(q1+q2+q3)
 *     | sin(q0) * (l2 * cos(q1+q2) + l3 * cos(q1+q2+q3))                  | l3 * sin(q0) * cos(q1+q2+q3)
 *     | -(l2 * sin(q2+q3) + l3 * sin(q1+q2+q3))                           | -l3 * sin(q1+q2+q3)
 *     | 1                                                                 | 1
 *  // | 0                                                                 | 0                                                              // beta
 */

#include "ros/ros.h"
#include <dof4_robot_arm/Value2Robot.h>
#include "HerkuleX.h"
#include <unistd.h>

#include <vector>
#include <map>

#include <unistd.h>
#include <iostream>
#include <cmath>
// #include <Eigen/Eigen>
#include <Eigen/Dense>

// Motor Num
#define JointNum 4

// Motor Id
#define M1 0
#define M2 1
#define M3 2
#define M4 3

// Motor Val
#define Dplaytime 3000/11.2
#define Dqval 30
#define LEDg 0x04
#define LEDb 0x08
#define LEDr 0x10

// Unit Convert
#define INCH2MM 25.4
#define SEC2HZ  1000/11.2       // sec -> motor control hz
#define PI 3.14159265
#define rad2deg 180/PI
#define deg2rad PI/180

// Robot Param
#define l0 91.0336		// 1.472 + 2.112 inch -> mm
#define l1 82.5246
#define l2 82.5246 
#define l3 71.8          	// line 100

// Steps for integration (d_q -> q)
#define STEPS 1000		//  millisec

using namespace std;
using namespace Eigen;

HerkuleX motor;

bool init(){
//    HerkuleX motor;
    int motor_id[JointNum] = {M1, M2, M3, M4};

    map<int,float> init_motor;
    
    for (int i = 0; i < JointNum; i++) {
        motor.TorqueOn(motor_id[i]);
	init_motor[motor_id[i]] = Dqval;
    }

    motor.moveAngle(init_motor, Dplaytime, HERKULEX_LED_GREEN);
    
    return true;
}

float d_p_func(float coeff1, float coeff2, float coeff3, float coeff4, float coeff5, float t){
    float d_p =  5*coeff1*pow(t,4) + 4*coeff2*pow(t,3) + 3*coeff3*pow(t,2) + 2*coeff4*t + coeff5;
    return d_p;
}

bool arm_control(dof4_robot_arm::Value2Robot::Request &req, 
                 dof4_robot_arm::Value2Robot::Response &res) 
{
    // get HerkuleX motor class
//    HerkuleX motor;

    // initialize motor id
    int motor_id[JointNum] = {M1, M2, M3, M4};

    // q values to be sent to motors
    map<int, float> q_motor;

    // get current angle from motor
    float q[JointNum] = {0, 0, 0, 0};

    for (int i = 0; i < JointNum; i++) {
        q[i] = motor.getAngle(motor_id[i]) * deg2rad;
        cout << i << "th motor angle = " << q[i] * rad2deg << endl;
    }

    // q dot, p dot
    Vector4f d_p(4);		// d_x, d_y, d_z, d_alp
    Vector4f d_q(4);		// d_q0, d_q1, d_q2, d_q3

    // Jacobian for d_q 
    Matrix4f J;

    // initial point (x, y, z : mm / alpha : degree)
    float x0 = ((l1*sin(q[1]) + l2*sin(q[1]+q[2]) + l3*sin(q[1]+q[2]+q[3]))*cos(q[0]));
    float y0 = (l1*sin(q[1]) + l2*sin(q[1]+q[2]) + l3*sin(q[1]+q[2]+q[3]))*sin(q[0]);
    float z0 = l1*cos(q[1]) + l2*cos(q[1]+q[2]) + l3*cos(q[1]+q[2]+q[3]);
    float alp0 = (q[1] + q[2] + q[3]);

    // Desired point (x, y, z : mm / alpha : degree)
    float xf = req.x;
    float yf = req.y;
    float zf = req.z;
    float alpf = req.alp;
    float t0 = req.t0;
    float tf = req.tf;
    // float betf = req.beta;
    alpf = alpf * deg2rad;

    // Path Planning
    // coefficients of path(linear) equation
    VectorXf coeff_x(6);
    VectorXf coeff_y(6);
    VectorXf coeff_z(6);
    VectorXf coeff_alp(6);

    // Matrix of time(t0 and tf) for solving path(linear) equation
    MatrixXf T(6, 6);
    
    // Matrix of initial and final points
    VectorXf i_f_x(6);
    VectorXf i_f_y(6);
    VectorXf i_f_z(6);
    VectorXf i_f_alp(6);

    // Initial and final point
    i_f_x   << x0, 0, 0, xf, 0, 0;
    i_f_y   << y0, 0, 0, yf, 0, 0;
    i_f_z   << z0, 0, 0, zf, 0, 0;
    i_f_alp << alp0, 0, 0, alpf, 0, 0;

    // T matrix
    T <<  pow(t0,5),    pow(t0,4),    pow(t0,3),   pow(t0,2), t0, 1,
          5*pow(t0,4),  4*pow(t0,3),  3*pow(t0,2), 2*t0,      1,  0,
	  20*pow(t0,3), 12*pow(t0,2), 6*t0,        2,         0,  0,
          pow(tf,5),    pow(tf,4),    pow(tf,3),   pow(tf,2), tf, 1,
          5*pow(tf,4),  4*pow(tf,3),  3*pow(tf,2), 2*tf,      1,  0,
          20*pow(tf,3), 12*pow(tf,2), 6*tf,        2,         0,  0;
    
    //find each coefficient according to the point assigned by using eigen.
    coeff_x = T.completeOrthogonalDecomposition().solve(i_f_x);
    coeff_y = T.completeOrthogonalDecomposition().solve(i_f_y);
    coeff_z = T.completeOrthogonalDecomposition().solve(i_f_z);
    coeff_alp = T.completeOrthogonalDecomposition().solve(i_f_alp);

    // angle integration for angle to move (d_p -> d_q -> q)
    // time step for integration
    int   moving_steps = (tf - t0) * STEPS;
    cout << "moving steps = " << moving_steps << endl;
    float time_step = 1.0/STEPS;
    cout << "time step = " << time_step << endl;

    for (int i = 0; i < moving_steps; i++) {
        // Jacobian J
        // x, y, z, alpha & q0, q1, q2, q3 -> 4 x 4 Jacobian symmetric
        J(0, 0) = -sin(q[0]) * ((l1 * sin(q[1])) + (l2 * sin(q[1] + q[2])) + (l3 * sin(q[1] + q[2] + q[3])));
        J(1, 0) =  cos(q[0]) * ((l1 * sin(q[1])) + (l2 * sin(q[1] + q[2])) + (l3 * sin(q[1] + q[2] + q[3])));
        J(2, 0) = 0;
        J(3, 0) = 0;

        J(0, 1) =  cos(q[0]) * ((l1 * cos(q[1])) + (l2 * cos(q[1] + q[2])) + (l3 * cos(q[1] + q[2] + q[3])));
        J(1, 1) =  sin(q[0]) * ((l1 * cos(q[1])) + (l2 * cos(q[1] + q[2])) + (l3 * cos(q[1] + q[2] + q[3])));
        J(2, 1) = -((l1 * sin(q[1])) + (l2 * sin(q[1] + q[2])) + (l3 * sin(q[1] + q[2] + q[3])));
        J(3, 1) = 1;

        J(0, 2) =  cos(q[0]) * ((l2 * cos(q[1] + q[2])) + (l3 * cos(q[1] + q[2] + q[3])));
        J(1, 2) =  sin(q[0]) * ((l2 * cos(q[1] + q[2])) + (l3 * cos(q[1] + q[2] + q[3])));
        J(2, 2) = -((l2 * sin(q[1] + q[2])) + (l3 * sin(q[1] + q[2] + q[3])));
        J(3, 2) = 1;

        J(0, 3) =  cos(q[0]) * l3 * cos(q[1] + q[2] + q[3]);
        J(1, 3) =  sin(q[0]) * l3 * cos(q[1] + q[2] + q[3]);
        J(2, 3) = -(l3 * sin(q[1] + q[2] + q[3]));
        J(3, 3) = 1;

        float d_x   = d_p_func(coeff_x(0),   coeff_x(1),   coeff_x(2),   coeff_x(3),   coeff_x(4),   (t0 + ((i+0.5) * time_step)));
        float d_y   = d_p_func(coeff_y(0),   coeff_y(1),   coeff_y(2),   coeff_y(3),   coeff_y(4),   (t0 + ((i+0.5) * time_step)));
        float d_z   = d_p_func(coeff_z(0),   coeff_z(1),   coeff_z(2),   coeff_z(3),   coeff_z(4),   (t0 + ((i+0.5) * time_step)));
        float d_alp = d_p_func(coeff_alp(0), coeff_alp(1), coeff_alp(2), coeff_alp(3), coeff_alp(4), (t0 + ((i+0.5) * time_step)));

        // d_p for Jacobian caculation 
        d_p(0) = d_x;
        d_p(1) = d_y;
        d_p(2) = d_z;
        d_p(3) = d_alp;
	
        // find d_q from Jacobian using eigen
        d_q = J.completeOrthogonalDecomposition().solve(d_p);

	// do integration
	for (int j =0; j < JointNum; j++) {
	    q[j] = q[j] + (d_q(j) * time_step);
	}
    }

    cout << "[Send data to motor]" << endl;
    // put motor values into map	
    for (int j = 0; j < JointNum; j++) {
        q_motor[motor_id[j]] = q[j]*rad2deg;
        // debug
        cout << j << "th desired q_val to motor: " << q_motor[j] << endl;
    }

    // calculate robot play time
    int playtime = (tf - t0) * SEC2HZ;

    // send integrated q values to motors.
    motor.moveAngle(q_motor, playtime, HERKULEX_LED_BLUE);
    cout << "robot moving..." << endl;
    ros::Duration(tf - t0).sleep();

    float q_check[4];
/*
    cout << "[debug]" << endl;
    for (int i = 0; i < JointNum; i ++) {
        q_check[i] = motor.getAngle(motor_id[i]) * deg2rad;
	cout << i << "th q_val from motor: " << q_check[i] * rad2deg << endl;
    }
*/
    res.cur_x = ((l1*sin(q_check[1]) + l2*sin(q_check[1]+q_check[2]) + l3*sin(q_check[1]+q_check[2]+q_check[3]))*cos(q_check[0]));
    res.cur_y = (l1*sin(q_check[1]) + l2*sin(q_check[1]+q_check[2]) + l3*sin(q_check[1]+q_check[2]+q_check[3]))*sin(q_check[0]);
    res.cur_z = l1*cos(q_check[1]) + l2*cos(q_check[1]+q_check[2]) + l3*cos(q_check[1]+q_check[2]+q_check[3]);
    res.cur_alp = (q_check[1] + q_check[2] + q_check[3]) * rad2deg;

    for (int i = 0; i < JointNum; i++){
        motor.setLed(motor_id[i], HERKULEX_LED_GREEN);
    }
    res.result = 1;
    
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dof4robot_control");
    ros::NodeHandle n;

    bool initialized = false;

    if(!initialized){
        initialized = init();
    }

    ros::ServiceServer service = n.advertiseService("Cmd2Dof4Robot", arm_control);
    ROS_INFO("Ready to move dof4robot arm.");
    ros::spin();

    return 0;
}
