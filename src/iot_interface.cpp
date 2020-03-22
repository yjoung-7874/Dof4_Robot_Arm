// ROS node get data from server and request to motor.
#include "ros/ros.h"
#include "center_bridge/MotorValue.h"
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
    ros::init(argc, argv, "center_motor_bridge");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<center_bridge::MotorValue>("Motor_Value");

    center_bridge::MotorValue srv;

    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;

    char to_server[257] = "cmd received\n";
    char from_server[1024];

    char *pch;

    int NumOfMotor, NumOfLED;
    int motorID, motorVal;
    int ledID, ledVal;

    map<int, int> tmp_led_val = {
                                 {0,0x10}, {1,0x08}, {2,0x10}, {3,0x10}, {4,0x08},
                                 {5,0x10}, {6,0x08}, {7,0x10}, {8,0x08}, {9,0x10}
                                };

    map<int, int> tmp_motor_val = {
                                   {0,100}, {1,200}, {2,300}, {3,400}, {4,500},
                                   {5,600}, {6,700},{7,800},{8,900},{9,1000}
                                  };

    string type;
    string tmp;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }

    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, "119.67.210.8", &serv_addr.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }

    for(int i =0 ; i< 10; i++){
        srv.request.job_description.push_back(0);
        srv.request.id.push_back(i);
        srv.request.motor_value.push_back(100);
        srv.request.playtime.push_back(0);
        srv.request.led.push_back(0x08);
    }

    if(client.call(srv)){
        ROS_INFO("Received = %ld", (long int)srv.response.result);
//        srv.request.job_description.clear();
//        srv.request.id.clear();
//        srv.request.motor_value.clear();
//        srv.request.playtime.clear();
//        srv.request.led.clear();
    } else {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    //main loop receiving cmd from hive
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
            cout << "Number of Motors : " << std::string(pch) << endl;
	    NumOfMotor = atoi(pch);
            if(NumOfMotor > 1){
		for(int i =0; i < NumOfMotor; i++) {
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


