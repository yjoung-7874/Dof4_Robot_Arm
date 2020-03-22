#include <iostream>
#include <cstring>

#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

#include <unistd.h>
#include <sys/signal.h>
#include <sys/types.h>

#include "HerkuleX.h"

unsigned char readbuf[255];

HerkuleX::HerkuleX() {
    this->Open();
}

HerkuleX::~HerkuleX() {
    this->Close();
}

bool HerkuleX::Open()
{
    struct termios newtio;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd == -1) {
    	std::cout << "Failed to open TTYUSB0" << std::endl;
        return false;
    }

    memset( &newtio, 0, sizeof(newtio) );

    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN]  = 0;

    tcflush (fd, TCIFLUSH );			//reset modem
    tcsetattr(fd, TCSANOW, &newtio );	//save setting

    //std::cout << "Serial Open! " << fd << std::endl;

    return true;
}

bool HerkuleX::Close() {
	if (fd != 0) {
		close(fd);
		//std::cout << "Serial Close! " << fd << std::endl;
	}
}

void HerkuleX::sendPacket(class DataPacket* buf){
	write(fd, buf, buf->packetSize);
}

int HerkuleX::receivePacket() {
	return read(fd, readbuf, sizeof(readbuf));
}

unsigned char HerkuleX::getChksum1(class DataPacket * buf){

    unsigned char CHksum1;
    CHksum1 = (buf->packetSize)^(buf->pID)^(buf->cmd);

    for(unsigned char i=0; i<buf->packetSize-7; i++ )
	CHksum1 ^= buf->data[i];

    return CHksum1;
}

unsigned char HerkuleX::getChksum2(unsigned char chksum1){

    unsigned char CHksum2;
    CHksum2 = ~chksum1 & 0xFE;

    return CHksum2;
}//

void HerkuleX::TorqueOn(int id){
    this->packet.packetSize = MIN_PACKET_SIZE + 3;
    this->packet.pID = id;
    this->packet.cmd = CMD_RAM_WRITE;

    this->packet.data[0] = 52;		//address
    this->packet.data[1] = 1;		//data length
    this->packet.data[2] = 0x60;	//torque on

    //CheckSum
    this->packet.chksum1 = this->getChksum1(&this->packet);
    this->packet.chksum2 = this->getChksum2(this->packet.chksum1);
    this->packet.chksum1 &= CHKSUM_MASK;
    this->packet.chksum2 &= CHKSUM_MASK;

    this->sendPacket(&this->packet);
    //std::cout<<"setHekulex["<<id<<"] Torque on : OK\n";
}

void HerkuleX::TorqueOff(int id){

    this->packet.packetSize = MIN_PACKET_SIZE + 3;
    this->packet.pID = id;
    this->packet.cmd = CMD_RAM_WRITE;

    this->packet.data[0] = 52;		//address
    this->packet.data[1] = 1;		//data length
    this->packet.data[2] = 0x00;	//torque free;

    //CheckSum
    this->packet.chksum1 = this->getChksum1(&this->packet);
    this->packet.chksum2 = this->getChksum2(this->packet.chksum1);
    this->packet.chksum1 &= CHKSUM_MASK;
    this->packet.chksum2 &= CHKSUM_MASK;

    this->sendPacket(&this->packet);

    //std::cout<<"setHekulex["<<id<<"] Torque off: OK\n";
}

void HerkuleX::movePos(int id, int pos, int playtime, int led){

    if(playtime<=0x00)
	playtime = 0x00;
    if(playtime >= 0xFE)
	playtime = 0xFE;

    if(id<=0x00)
	id = 0x00;
    if(id >= 0xFE)
	id = 0xFE;

    if(pos<=0)
	pos = 0;
    if(pos>=1024)
	pos = 1024;

    this->packet.header[0] = HEADER;
    this->packet.header[1] = HEADER;

    this->packet.packetSize = MIN_PACKET_SIZE + 5;
    this->packet.pID = BROADCAST_ID;
    this->packet.cmd =  CMD_I_JOG;

    this->packet.data[0] = LSB(pos);		//LSB
    this->packet.data[1] = MSB(pos);		//MSB
    this->packet.data[2] = led;				//SET
    this->packet.data[3] = id; 				//ID
    this->packet.data[4] = playtime;		//playTime

    //CheckSum
    this->packet.chksum1 = this->getChksum1(&this->packet);
    this->packet.chksum2 = this->getChksum2(this->packet.chksum1);
    this->packet.chksum1 &= CHKSUM_MASK;
    this->packet.chksum2 &= CHKSUM_MASK;

    this->sendPacket(&this->packet);

    //std::cout<< "Succeed to move!\n" ;
}

void HerkuleX::movePos(std::map<int, int> motor_values, int playtime, int led){

	if(playtime<=0x00)
		playtime = 0x00;
	if(playtime >= 0xFE)
		playtime = 0xFE;

	this->packet.header[0] = HEADER;
	this->packet.header[1] = HEADER;

    // iterator
    std::map<int, int>::const_iterator iterator;
    int idx = 0, id, pos;

    for (iterator = motor_values.begin(); iterator != motor_values.end(); iterator++) {
    	id = iterator->first;
    	pos = iterator->second;

    	if(id < 0 || id > 253) continue;

		if(pos<=0)
			pos = 0;
		if(pos>=1024)
			pos = 1024;

		this->packet.data[++idx] = LSB(pos);		//LSB
		this->packet.data[++idx] = MSB(pos);		//MSB
		this->packet.data[++idx] = led;				//SET
		this->packet.data[++idx] = id; 				//ID
    }

    if (idx == 0) return;

    this->packet.data[0] = playtime;		//playTime
    this->packet.packetSize = MIN_PACKET_SIZE + idx + 1;
    this->packet.pID = BROADCAST_ID;
    this->packet.cmd =  CMD_S_JOG;

    //CheckSum
    this->packet.chksum1 = this->getChksum1(&this->packet);
    this->packet.chksum2 = this->getChksum2(this->packet.chksum1);
    this->packet.chksum1 &= CHKSUM_MASK;
    this->packet.chksum2 &= CHKSUM_MASK;

    this->sendPacket(&this->packet);

    //std::cout<< "Succeed to move!\n" ;
}


void HerkuleX::setLed(int id, int led){
//--------------------------------------------------------------------------
    this->packet.packetSize = MIN_PACKET_SIZE + 3;
    this->packet.pID = id;
    this->packet.cmd = CMD_RAM_WRITE;

    this->packet.data[0] = 53;		//address
    this->packet.data[1] = 1;		//data length
    this->packet.data[2] = led;		//led value

    //CheckSum
    this->packet.chksum1 = this->getChksum1(&this->packet);
    this->packet.chksum2 = this->getChksum2(this->packet.chksum1);
    this->packet.chksum1 &= CHKSUM_MASK;
    this->packet.chksum2 &= CHKSUM_MASK;

    this->sendPacket(&this->packet);

	//std::cout<< "color of led is changed!\n";

}

void HerkuleX::moveAngle(int id, float angle, int playtime, int led){

	//Degree = Position Raw Data X 0.325
	int angValue;

	if(angle>320)
		angle=320;
	if(angle<0)
		angle=0;

	angValue = (unsigned short int)((float)angle / (float)0.325);

	if(angValue<0)
		angValue=0;
	if(angValue>1024)
		angValue = 1024;

	this->movePos(id,angValue,playtime,led);
}

void HerkuleX::clear(int id){
    this->packet.packetSize = MIN_PACKET_SIZE + 3;
    this->packet.pID = id;
    this->packet.cmd = CMD_RAM_WRITE;

    this->packet.data[0] = 48;		//address
    this->packet.data[1] = 1;		//data length
    this->packet.data[2] = 0x00;		//led value

     //CheckSum
    this->packet.chksum1 = this->getChksum1(&this->packet);
    this->packet.chksum2 = this->getChksum2(this->packet.chksum1);
    this->packet.chksum1 &= CHKSUM_MASK;
    this->packet.chksum2 &= CHKSUM_MASK;

	this->sendPacket(&this->packet);
}

void HerkuleX::reboot(int id){
    this->packet.packetSize = MIN_PACKET_SIZE;
    this->packet.pID = id;
    this->packet.cmd = CMD_REBOOT;	//CMD_REBOOT

    //CheckSum
    this->packet.chksum1 = this->getChksum1(&this->packet);
    this->packet.chksum2 = this->getChksum2(this->packet.chksum1);
    this->packet.chksum1 &= CHKSUM_MASK;
    this->packet.chksum2 &= CHKSUM_MASK;

    this->sendPacket(&this->packet);  
}

void HerkuleX::turn(int id, int speed, int playtime, int led){

    if(playtime >= 254)
	playtime = 254;

    if(id >= 254)
	id = 254;

    if (speed < 0) {
	speed = (-1) * speed;
	speed |= 0x4000;
    }

    this->packet.packetSize = MIN_PACKET_SIZE+5;
    this->packet.pID = BROADCAST_ID;
    this->packet.cmd = CMD_I_JOG;	//I_JOG cmd

    this->packet.data[0] = LSB(speed);
    this->packet.data[1] = MSB(speed);

    this->packet.data[2] = led|(1<<1);		//led color
    this->packet.data[3] = id;			//survo id
    this->packet.data[4] = playtime;	//play time

    //CheckSum
    this->packet.chksum1 = this->getChksum1(&this->packet);
    this->packet.chksum2 = this->getChksum2(this->packet.chksum1);
    this->packet.chksum1 &= CHKSUM_MASK;
    this->packet.chksum2 &= CHKSUM_MASK;

    this->sendPacket(&this->packet);

}

int HerkuleX::getPos(int id){
	int position;

	if (id < 0 || id > 253) return -1;

	this->packet.packetSize = MIN_PACKET_SIZE+2;
	this->packet.pID = id;
	this->packet.cmd = CMD_RAM_READ;

	this->packet.data[0] = 0x3A;
	this->packet.data[1] = 0x02;

	//CheckSum
	this->packet.chksum1 = this->getChksum1(&this->packet);
	this->packet.chksum2 = this->getChksum2(this->packet.chksum1);
	this->packet.chksum1 &= CHKSUM_MASK;
	this->packet.chksum2 &= CHKSUM_MASK;

	this->sendPacket(&this->packet);

	usleep(10000);

	if (receivePacket() < 13) return -1;

	position = ((readbuf[9]&0xff) | ((readbuf[10]&0x03) << 8));
	return position;
}

float HerkuleX::getTurnSpeed(int id){
	int speed;

	if (id < 0 || id > 253) return -1;

	this->packet.packetSize = MIN_PACKET_SIZE+2;
	this->packet.pID = id;
	this->packet.cmd = CMD_RAM_READ;

	this->packet.data[0] = 0x40;
	this->packet.data[1] = 0x02;

	//CheckSum
	this->packet.chksum1 = this->getChksum1(&this->packet);
	this->packet.chksum2 = this->getChksum2(this->packet.chksum1);
	this->packet.chksum1 &= CHKSUM_MASK;
	this->packet.chksum2 &= CHKSUM_MASK;

	this->sendPacket(&this->packet);

	usleep(10000);

	if (receivePacket() < 13) return -1;

    speed = ((readbuf[10] & 0x03) << 8) | (readbuf[9] & 0xFF);
    if ((readbuf[10] & 0x40) == 0x40){
    	speed = (1024 - speed) * -1;
    }

	return speed;
}


float HerkuleX::getAngle(int id){
    float angle;

    int pos = this->getPos(id);
    if (pos < 0) return 0;
	angle = (pos - 512) * 0.325f;

    return angle;
}
