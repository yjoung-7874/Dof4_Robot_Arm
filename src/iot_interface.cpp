// ROS node get data from server and request to motor.
#include "ros/ros.h"
#include <dof4_robot_arm/Value2Robot.h>
#include <iostream>
#include <vector>
#include <sstream>

#include <unistd.h>
#include <sys/socket.h>
#include <string>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <cstring>

#define DEFAULT_PTIME 200
#define SINGLE_MOTOR 1
#define MULTI_MOTORS 2 
#define PORT 15003

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "iot_interface");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<dof4_robot::Value2Robot>("Cmd2Dof4Robot");

    // srv to robot
    dof4_robot_arm::Value2Robot srv;
    float desired[6]= {0, 0, 0, 0, 0, 0};

    // server
    char to_server[257] = "cmd received\n";
    char from_server[1024];

    // string splitter
    char *pch;

    string type;
    string tmp;

    // values for socket 
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }
    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, "1.222.20.246", &serv_addr.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }

    // main loop receiving cmd from hive
    while(ros::ok()){

        //recv msg from server
        if(recv(sock, from_server, sizeof(from_server)/sizeof(*from_server), 0) < 0)
            std::cout<<"recv failed";
        else
            printf("from server : %s", from_server);

	//send response to server
        if(send(sock, to_server, strlen(to_server), 0) < 0)
            std::cout<<"send failed"<<endl;
        else {
            printf("cmd = %s", to_server);
            usleep(500);
        }

        // split cmd string (if cmd type is mode, save cmd and value)
        pch = strtok(from_server," ");
        type = std::string(pch);

        if(type == "move") {
            pch = strtok(NULL," ");
	    float x_ = atof(pch);
	    pch = strtok(NULL," ");
	    float y_ = atof(pch);
	    pch = strtok(NULL," ");
	    float z_ = atof(pch);
            pch = strtok(NULL," ");
	    float alp_ = atof(pch);
            pch = strtok(NULL," ");
	    float t0_ = atof(pch);
	    pch = strtok(NULL," ");
	    float tf_ = atof(pch);

	    cout << "x   = " << x_ << endl;
            cout << "y   = " << y_ << endl;
            cout << "z   = " << z_ << endl;
            cout << "alp = " << alp_ << endl;
            cout << "t0  = " << t0_ << endl; 
            cout << "tf  = " << tf_ << endl;

	    srv.request.x = x_;
	    srv.request.y = y_;
	    srv.request.z = z_;
	    srv.request.alp = alp_;
	    srv.request.t0 = t0_;
	    srv.request.tf = tf_;
	} else {
            pch = strtok(NULL," ");
            motorID = atoi(pch);
            cout << "motor id = " << motorID << endl;
            srv.request.job_description.at(motorID) = MULTI_MOTORS;
            srv.request.id.at(motorID) = motorID; // put motor id to request

            pch = strtok(NULL," ");
            motorVal = atoi(pch);
            cout << "motor value = " << motorVal << endl;
            srv.request.motor_value.at(motorID) = motorVal;
            tmp_motor_val.at(motorID) = motorVal;
            srv.request.led.at(motorID) = tmp_led_val.at(motorID);
            srv.request.playtime.at(motorID) = DEFAULT_PTIME;
	}

	    if(client.call(srv)){
	        ROS_INFO("Received = %ld", (long int)srv.response.result);
	    } else {
	        ROS_ERROR("Failed to call service");
	        return 1;
	    }

        }

        if(type == "light"){
            pch = strtok(NULL," ");
            cout << "Number of LEDs : " << std::string(pch) << endl;
            NumOfLED = atoi(pch);
            if(NumOfLED > 1){
                for(int i =0; i < NumOfLED; i++) {
                    pch = strtok(NULL," ");
                    ledID = atoi(pch);
                    cout << "LED id = " << ledID << endl;
                    srv.request.job_description.at(ledID) = MULTI_MOTORS;
                    srv.request.id.at(ledID) = ledID; // put led id to request

                    pch = strtok(NULL," ");
                    ledVal = atoi(pch);
                    cout << "led value = " << ledVal << endl;
                    srv.request.led.at(ledID) = ledVal;
                    tmp_led_val.at(ledID) = ledVal;
                    srv.request.motor_value.at(ledID) = tmp_led_val.at(ledID);
                    srv.request.playtime.at(ledID) = DEFAULT_PTIME;
                }
            } else {
                pch = strtok(NULL," ");
                ledID = atoi(pch);
                cout << "LED id = " << ledID << endl;
                srv.request.job_description.at(ledID) = MULTI_MOTORS;
                srv.request.id.at(ledID) = ledID; // put led id to request

                pch = strtok(NULL," ");
                ledVal = atoi(pch);
                cout << "led value = " << ledVal << endl;
                srv.request.led.at(ledID) = ledVal;
                tmp_led_val.at(ledID) = ledVal;
                srv.request.motor_value.at(ledID) = tmp_led_val.at(ledID);
                srv.request.playtime.at(ledID) = DEFAULT_PTIME;
	    }
            if(client.call(srv)){
                ROS_INFO("Received = %ld", (long int)srv.response.result);
            } else {
                ROS_ERROR("Failed to call service");
                return 1;
            }

        }

/*        for(int i =0 ; i< 10; i++){
            srv.request.job_description.push_back(0);
            srv.request.id.push_back(i);
            srv.request.motor_value.push_back(0);
            srv.request.playtime.push_back(0);
            srv.request.led.push_back(0);

	    srv.request.job_description.push_back(1); 
	    srv.request.id.push_back(i);
	    srv.request.motor_value.push_back(i*50);
	    srv.request.playtime.push_back(100);

	    if(i%2 == 1)
	        srv.request.led.push_back(0x10);
	    else
	        srv.request.led.push_back(0x08);
        }*/
    }


/*    if(client.call(srv)){
	ROS_INFO("Received = %ld", (long int)srv.response.result);
    } else {
	ROS_ERROR("Failed to call service");
	return 1;
    }*/
    return 0;
}


