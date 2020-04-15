#ifndef __HERKULEX
#define __HERKULEX

#include <map>
using namespace std;

//HEADER
#define HEADER                              0xFF

//SIZE
#define MIN_PACKET_SIZE                     7
#define MIN_ACK_PACKET_SIZE                 9
#define MAX_PACKET_SIZE                     223
#define MAX_DATA_SIZE                       (MAX_PACKET_SIZE-MIN_PACKET_SIZE)

//ID
#define MAX_ID                              0xFD
#define BROADCAST_ID                        0xFE

//CMD - Request Packet
#define CMD_EEP_WRITE                       0x01
#define CMD_EEP_READ                        0x02
#define CMD_RAM_WRITE                       0x03
#define CMD_RAM_READ                        0x04
#define CMD_RW_DATA_ADDR_IDX                7
#define CMD_RW_DATA_LEN_IDX                 8
#define CMD_I_JOG                           0x05
#define CMD_I_JOG_STRUCT_SIZE               5
#define CMD_I_JOG_MAX_DRS                   (MAX_DATA_SIZE/CMD_I_JOG_STRUCT_SIZE)
#define CMD_S_JOG                           0x06
#define CMD_S_JOG_STRUCT_SIZE               4
#define CMD_S_JOG_MAX_DRS                   (MAX_DATA_SIZE/CMD_S_JOG_STRUCT_SIZE)
#define CMD_STAT                            0x07
#define CMD_ROLLBACK                        0x08
#define CMD_REBOOT                          0x09

#define CMD_MIN                             (CMD_EEP_WRITE)
#define CMD_MAX                             (CMD_REBOOT)

#define CHKSUM_MASK                         0xFE

#define LSB(data) ((unsigned char)(data))
#define MSB(data) ((unsigined char)((unsigned int)(data)>>8)&0xFF)

// LED
#define HERKULEX_LED_RED	0x10
#define HERKULEX_LED_GREEN	0x04
#define HERKULEX_LED_BLUE	0x08

class DataPacket{
public :
    unsigned char header[2];
    unsigned char packetSize;
    unsigned char pID;
    unsigned char cmd;
    unsigned char chksum1;
    unsigned char chksum2;
    unsigned char data[216];
    DataPacket(){
        this->header[0] = HEADER;
        this->header[1] = HEADER;
    }
};

class HerkuleX {
public:
    HerkuleX();	
    ~HerkuleX();

    //void setAckPolicy(int id, bool value);	//응답 정책 설정
    //void setLEDPolicy(int id, bool value);	//LED 점멸 정책 설정
    void clear(int id);				//에러 삭제

    void TorqueOn(int id);		//torque on 
    void TorqueOff(int id);		//torque off

    void turn(int id, int speed, int playtime=60, int led=0);	//Motor Turn 속도 제어
    float getTurnSpeed(int id);     				//turn 속도  position변화량/11.2ms 		

    void movePos(int id, int pos, int playtime=60, int led=0);	//Motor의 Position 제어
    void movePos(map<int, int> motor_values, int playtime=60, int led=0);
    int getPos(int id);						//Motor의 Position 값 리턴

    void moveAngle(int id, float angle, int playtime=60, int led=0);	//Motor의 각도 제어
    void moveAngle(map<int, float> motor_angles, int playtime, int led);
    float getAngle(int id);						//Motor의 각도 값 리턴

    void setLed(int id, int led);			//Motor LED
    void reboot(int id);				//Motor

private:
    bool Open();		//serial port open
    bool Close();		//serial port close

    unsigned char getChksum1(class DataPacket * buf);	//Check sum1
    unsigned char getChksum2(unsigned char chksum1);	//Check sum2

    void sendPacket(class DataPacket * buf);			//Packet
    int receivePacket();

    int fd;				//file description
    //data package class
    class DataPacket packet;
};


#endif
