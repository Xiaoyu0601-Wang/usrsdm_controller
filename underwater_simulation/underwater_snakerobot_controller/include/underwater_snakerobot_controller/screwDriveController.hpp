#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "serialPort.hpp"

class ScrewDriveController {
  public:
    union DataUnionF {
      float as_float;
      unsigned char as_byte[4];
    };

    SerialPort usbCom;
    int link_n;

    ScrewDriveController()
    {
    	if (usbCom.Open("/dev/ttyUSB1") == true) {
        printf("Succeeded to connect to screw drives!\n");
    	}
      else {
        printf("Can not connect to screw drives!\n");
      }

    	link_n = 2; // by default

    	SetBuzzer(1,6);
    }
    ~ScrewDriveController()
    {
      SetBuzzer(2,6);

      for(int id=1; id<=link_n; id++)
      {
        SetVelocity(id, 0);
      }
    	usbCom.Close();
    }

    void SetBuzzer(int type, int pitch)
    {
        //�R�}���h�𐶐�
        unsigned char command[9];
        command[0] = 0xFF;                      //�ŏ���2�p�P�b�g��0xFF�Œ�
        command[1] = 0xFF;                      //
        command[2] = 0x00;                      //ID:0
        command[3] = type;                      //INSTRUCTION:�f�[�^�̏�������
        command[4] = 25;                        //PARAMETER1:�������݃A�h���X�͖ڕW�p����
        command[5] = pitch;                     //PARAMETER2:�ڕW�p2�p�P�b�g(L->H)
        command[6] = 0x00;                      //PARAMETER3:
        command[7] = 0x00;                      //PARAMETER4:�p���x2�p�P�b�g(L->H)
        command[8] = 0x00;                      //CHECKSUM:
        for(int i=2; i<=7; i++) {                //�a�������̂�ID����
            command[8] += command[i];           //
        }                                       //
        command[8] = ~command[8];               //�Ō��ɔے����Ƃ�

        //�C���X�g���N�V�����p�P�b�g�̏�������
        usbCom.Write(command, 9);

        // return;
    }

    void SetVelocity(int id, float omega_ref) {
    	int range_flag = 0;
    	// Check the range of omega
    	if (omega_ref > 100) {
    		range_flag = 1;
    		omega_ref = 100;
    	}
    	if (omega_ref < -100) {
    		range_flag = 1;
    		omega_ref = -100;
    	}

    	//�ڕW�p�x���r�b�g���Ƃɕ�����(�r�b�g���Z�q���g���Ă���)

    	union DataUnionF TxData;
    	TxData.as_float = omega_ref;

    	unsigned char id_char = id;

    	//�R�}���h�𐶐�
    	unsigned char command[9];
    	command[0] = 0xFF;						//�ŏ���2�p�P�b�g��0xFF�Œ�
    	command[1] = 0xFF;						//
    	command[2] = id_char;					//ID:1
    	command[3] = 0x11;						//INSTRUCTION:�f�[�^�̏�������
    	command[4] = TxData.as_byte[0];			//PARAMETER1:�������݃A�h���X�͖ڕW�p����
    	command[5] = TxData.as_byte[1];			//PARAMETER2:�ڕW�p2�p�P�b�g(L->H)
    	command[6] = TxData.as_byte[2];			//PARAMETER3:
    	command[7] = TxData.as_byte[3];			//PARAMETER4:�p���x2�p�P�b�g(L->H)
    	command[8] = 0x00;						//CHECKSUM:
    	for(int i=2; i<=7; i++){				//�a�������̂�ID����
    		command[8] += command[i];			//
    	}										//
    	command[8] = ~command[8];				//�Ō��ɔے����Ƃ�

    	//�C���X�g���N�V�����p�P�b�g�̏�������
    	usbCom.Write(command, 9);

    	//mySerial.Read2(buf,9);

    	//�o�b�t�@���������Ă���
    	// mySerial.ClearRXbuffer();

    	return;
    }

    double GetVelocity(int id)
    {
    	union DataUnionF RxData;

    	unsigned char id_char = id;

    	unsigned char command[9],buf[9];
    	command[0]=0xFF;				//�ŏ���2�p�P�b�g��0xFF�Œ�
    	command[1]=0xFF;				//
    	command[2]=id_char;				//ID:�f�t�H���g��1
    	command[3]=0x12;				//LENGTH:�����ڂ����ŏI���ڂ܂ł̍��ڐ�
    	command[4]=0x00;				//INSTRUCTION:�f�[�^�̓ǂݎ���
    	command[5]=0x00;				//PARAMETER1:�ǂݎ����A�h���X�͖ڕW�p����
    	command[6]=0x00;				//PARAMETER2:�ǂݎ����̂�2�A�h���X��
    	command[7]=0x00;				//
    	command[8]=0x00;				//CHECKSUM:
    	for(int i=2; i<=7; i++){		//�a�������̂�ID����
    		command[8]+=command[i];		//
    	}								//
    	command[8]=~command[8];			//�Ō��ɔے����Ƃ�

    	int c_error=0;
    	do{
    		usbCom.Write(command, 9);

    		//�X�e�[�^�X�p�P�b�g���M
    		usbCom.Read(buf, 9);

    		unsigned char temp = 0;
    		for(int i=2; i<=7; i++){
    			temp += buf[i];
    		}
    		if (temp != ~buf[8])
    		{
    			c_error = 1;
    		}
    	}while(c_error==0);

    	//�o�b�t�@���������Ă���
    	// mySerial.ClearRXbuffer();

    	//�߂��l���v�Z���ĕԂ�
    	for(int i=0; i<4; i++)
    	{
    		RxData.as_byte[i] = buf[i+4];
    	}
    	float omega = RxData.as_float;
    	return omega;
    }

};
