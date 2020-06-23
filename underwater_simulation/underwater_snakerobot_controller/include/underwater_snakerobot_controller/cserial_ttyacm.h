#ifndef CSERIAL_TTYACM_H_
#define CSERIAL_TTYACM_H_

#include <stdio.h>
#include <iostream>
//#include <boost/asio.hpp>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
using namespace std;

class CSerial_ttyACM
{
public:
	int fileHandle;
	struct termios Optiton;
	string PortName;

	bool b_open();
	bool write_oneByte(char data_in);
	bool write_string(string data_in);
	bool read_oneByte(char& data_out);
	int read_severalByte(char* data_out, int maxSize);
	void read_test();
	int read_toLast(string& recieved_Buffer_out);
	CSerial_ttyACM();
	~CSerial_ttyACM();
};

CSerial_ttyACM::CSerial_ttyACM()
{
	PortName = "/dev/ttyACM0";
}
CSerial_ttyACM::~CSerial_ttyACM()
{
	close(fileHandle);
}

bool CSerial_ttyACM::b_open()
{
	fileHandle = open(PortName.c_str(), O_RDWR);

	if (fileHandle < 0)
		return false;

	tcgetattr(fileHandle, &Optiton);				//gets the current options for the port
	cfsetispeed(&Optiton, B115200);
	cfsetospeed(&Optiton, B115200);
	//Optiton.c_cflag |= (CLOCAL | CREAD);		//all these set options for 8N1 serial operations
	//Optiton.c_cflag &= ~PARENB;
	//Optiton.c_cflag &= ~CSTOPB;
	//Optiton.c_cflag &= ~CSIZE;
	//Optiton.c_cflag |= CS8;
	//Set the new options for the port "NOW"
	Optiton.c_cc[VMIN] = 0;  //set max waiting time
	Optiton.c_cc[VTIME] = 1; //max waiting time = 100ms
	int status = tcsetattr(fileHandle, TCSANOW, &Optiton);
	if(status != 0)
		return false;

	tcflush(fileHandle, TCIOFLUSH);

	return true;
}

bool CSerial_ttyACM::write_oneByte(char data_in)
{
	char data[255];
	data[0] = data_in;
	if(write(fileHandle, data, 1) > 0)
		return true;
	else
		return false;
}

bool CSerial_ttyACM::write_string(string data_in)
{
	char data[255];
	for(unsigned int i=0; i<data_in.size(); i++)
	{
		if(data_in[i] == '\0') //in case: g++: string is end with \0
			break;
		data[i] = data_in[i];
		if(data_in[i] == '\n')
			break;
	}
	if(write(fileHandle, data, data_in.size()) > 0)
		return true;
	else
		return false;
}

bool CSerial_ttyACM::read_oneByte(char& data_out)
{
	char data[255];
	int count = read(fileHandle, data, 1);
	if(count == 1)
		data_out = data[0];
	else
		return false;
	return true;
}

int CSerial_ttyACM::read_severalByte(char* data_out, int maxSize)
{
	return read(fileHandle, data_out, maxSize);
}


void CSerial_ttyACM::read_test()
{
	char data[255];
	int count = read(fileHandle, data, 255);
	if(count > 0){
		for(int i=0; i<count; i++)
			printf("%c",data[i]);
		printf("\n");
	}
}

int CSerial_ttyACM::read_toLast(string& recieved_Buffer_out)
{
	int count = 0;
	char data[255];
	recieved_Buffer_out.clear();
	int read_MaxSize = 255;
	while(true)
	{
		int read_data_size = 0;
		read_data_size = read(fileHandle, data, read_MaxSize);
		if(read_data_size > 0)
		{
			recieved_Buffer_out.append(data, read_data_size);
			count += read_data_size;
		}
		if(read_data_size <= read_MaxSize)
			break;
	}
	return count;
}

#endif /* CSERAIL_TTYACM_H_ */
