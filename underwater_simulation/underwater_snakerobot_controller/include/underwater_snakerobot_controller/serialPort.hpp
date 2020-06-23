#pragma once

#include <math.h>

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include<cserial_ttyacm.h>

class SerialPort
{
  private:
    std::string port_name; // ex) "/dev/ttyACM0"
    int port_fd;

  public:
    struct termios Option;

    bool Open(const char* port) {
      port_name = port;
      return Open();
    }

    bool Open() {
      port_fd = open(port_name.c_str(), O_RDWR);// | O_NOCTTY | O_NONBLOCK | O_NDELAY);  //Open serial port
      // tcgetattr(port_fd, &Option);				//gets the current options for the port

      // Read in existing settings, and handle any error
      if(tcgetattr(port_fd, &Option) != 0)
      {
          printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      }

      cfsetispeed(&Option, B57600);
      cfsetospeed(&Option, B57600);

      Option.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
      Option.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
      Option.c_cflag |= CS8; // 8 bits per byte (most common)
      Option.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
      Option.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

      Option.c_lflag &= ~ICANON;
      Option.c_lflag &= ~ECHO; // Disable echo
      Option.c_lflag &= ~ECHOE; // Disable erasure
      Option.c_lflag &= ~ECHONL; // Disable new-line echo
      Option.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
      Option.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
      Option.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

      Option.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
      Option.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
///////////////////////////////////////////////////////////////
      // Option.c_cflag |= (CLOCAL | CREAD);		//all these set options for 8N1 serial operations
      // Option.c_cflag &= ~PARENB;
      // Option.c_cflag &= ~CSTOPB; //.c_cflag |= CSTOPB;//.c_cflag &= ~CSTOPB;            // 1 stop bit
      // Option.c_cflag &= ~CRTSCTS;           // Disable hardware flow control
      // Option.c_cflag &= ~CSIZE;
      // Option.c_cflag |= CS8;

      //Set the new options for the port "NOW"
      Option.c_cc[VMIN] = 0;  //set max waiting time
      Option.c_cc[VTIME] = 5; //max waiting time = 100ms
////////////////////////////////////////////////////////////////
      // Enable data to be processed as raw input
      // Option.c_lflag &= ~(ICANON | ECHO | ISIG);

      // Set the new attributes
      // int status = tcsetattr(port_fd, TCSANOW, &Option);
      // if(status != 0) {
      //   printf("USB can not be connected");
      //   return false;
      // }

      if (tcsetattr(port_fd, TCSANOW, &Option) != 0) {
          printf("USB can not be connected\n");
          return false;
      }
      tcflush(port_fd, TCIOFLUSH);

      if(port_fd < 0) {
        printf("USB can not be connected\n");
        return false;  //Port open error
      }
      if(port_fd > 0) {
        printf("USB is connected\n");
        // SetBuzzer(1,6);
        return true;
      }
    }

    void Close() {
      if(port_fd > 0) {
        close(port_fd);  //Close serial port
        port_fd = -1;
      }
    }

    int Read(unsigned char* buf, unsigned int buf_len) {
      if(port_fd < 0) {
        printf("USB reading is not working well");
        return -1;
      }

      return read(port_fd, buf, buf_len);  //Read device file and put the data into buf (max size is (buf_len) bytes)
                                           //Process stops here until data comes
    }

    int Write(unsigned char* data, unsigned int data_len) {
      if(port_fd < 0) {
        printf("USB writing is not working well");
        return -1;
      }

      if(write(port_fd, data, data_len) < 0) {
        printf("USB writing is not working well\n");
        return -1;
      }

      return 1;
    }
    // void SetBuzzer(int type, int pitch)
    // {
    //     //�R�}���h�𐶐�
    //     unsigned char command[9];
    //     command[0] = 0xFF;                      //�ŏ���2�p�P�b�g��0xFF�Œ�
    //     command[1] = 0xFF;                      //
    //     command[2] = 0x00;                      //ID:0
    //     command[3] = type;                      //INSTRUCTION:�f�[�^�̏�������
    //     command[4] = 25;                        //PARAMETER1:�������݃A�h���X�͖ڕW�p����
    //     command[5] = pitch;                     //PARAMETER2:�ڕW�p2�p�P�b�g(L->H)
    //     command[6] = 0x00;                      //PARAMETER3:
    //     command[7] = 0x00;                      //PARAMETER4:�p���x2�p�P�b�g(L->H)
    //     command[8] = 0x00;                      //CHECKSUM:
    //     for(int i=2; i<=7; i++) {                //�a�������̂�ID����
    //         command[8] += command[i];           //
    //     }                                       //
    //     command[8] = ~command[8];               //�Ō��ɔے����Ƃ�
    //
    //     //�C���X�g���N�V�����p�P�b�g�̏�������
    //     Write(command, 9);
    //
    //     // return;
    // }
};
