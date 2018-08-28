#ifndef MULTI_MODBUS_SLAVE_SOFTWARESERIAL_H
#define MULTI_MODBUS_SLAVE_SOFTWARESERIAL_H

/*
 
 The multislave SoftwareSerial allows you to communicate
 to any slave using the Modbus RTU protocol at the same time allowing you to host multiple slaves on a single device.
 New modifications include object based code.
 Host multiple slaves (2) in a single device.
 software SoftwareSerial implementation of slave.
 Tested on Mega 2560 and Esp8266( ESP12-E ).
 Thus can act as a hybrid modbus device (gateway) reading modbus data from one device and serving to another device.

 V1.0 completed on 20/08/2018 by Divyanshu Anand

 Credits:
 Based on SimpleModbusSlave  library by Juan Bester
 The crc calculation is based on the work published 
 by jpmzometa at 
 http://sites.google.com/site/jpmzometa/arduino-mbrt
 
 
 
 The functions implemented are functions 3 and 16.
 read holding registers and preset multiple registers
 of the Modbus RTU Protocol, to be used over the Arduino serial connection.
 
 This implementation DOES NOT fully comply with the Modbus specifications.
 
 Specifically the frame time out have not been implemented according
 to Modbus standards. The code does however combine the check for
 inter character time out and frame time out by incorporating a maximum
 time out allowable when reading from the message stream.
 
 These library of functions are designed to enable a program send and
 receive data from a device that communicates using the Modbus protocol.
 
 This code is for a Modbus slave implementing functions 3 and 16
 function 3: Reads the binary contents of holding registers (4X references)
 function 16: Presets values into a sequence of holding registers (4X references)
 
 All the functions share the same register array.
 
 Exception responses:
 1 ILLEGAL FUNCTION
 2 ILLEGAL DATA ADDRESS
 3 ILLEGAL DATA VALUE
 
 Note:  
 The Arduino serial ring buffer is 128 bytes or 64 registers.
 Most of the time you will connect the arduino to a master via serial
 using a MAX485 or similar.
 
 In a function 3 request the master will attempt to read from your
 slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
 and two BYTES CRC the master can only request 122 bytes or 61 registers.
 
 In a function 16 request the master will attempt to write to your 
 slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
 NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
 118 bytes or 59 registers.
 
 Using the FTDI converter ic the maximum bytes you can send is limited 
 to its internal buffer which is 60 bytes or 30 unsigned int registers. 
 
 Thus:
 
 In a function 3 request the master will attempt to read from your
 slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
 and two BYTES CRC the master can only request 54 bytes or 27 registers.
 
 In a function 16 request the master will attempt to write to your 
 slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
 NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
 50 bytes or 25 registers.
  
 Since it is assumed that you will mostly use the Arduino to connect to a 
 master without using a USB to Serial converter the internal buffer is set
 the same as the Arduino Serial ring buffer which is 128 bytes.
 
 The functions included here have been derived from the 
 Modbus Specifications and Implementation Guides
 
 http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
 http://www.modbus.org/docs/PI_MBUS_300.pdf
*/

#include "Arduino.h"
#include "SoftwareSerial.h"
#define BUFFER_SIZE 128
class MultiModbusSlaveSS
{
 public:  
    // function definitions
        void modbus_slaveconfigure(long baud, byte _slaveID,unsigned char _slaveID2, byte _TxEnablePin, unsigned int _holdingRegsSize);
        unsigned int modbus_slaveupdate(unsigned int *holdingRegs,unsigned int *holdingRegs2);
  
  private: 
        //private function     
        void SlaveexceptionResponse(unsigned char exception);
        unsigned int SlavecalculateCRC(unsigned char bufferSize); 
        void SlavesendPacket(unsigned char bufferSize);

        //private variables
        unsigned char Slaveframe[BUFFER_SIZE];
        unsigned int holdingRegsSize; // size of the register array 
        unsigned char SlavebroadcastFlag;
        unsigned char slaveID;
        unsigned char slaveID2;
        unsigned char Slavefunction;
        unsigned char SlaveTxEnablePin;
        unsigned int SlaveerrorCount;
        unsigned int SlaveT1_5; // inter character time out
        unsigned int T3_5; // Slaveframe delay

};

#endif
