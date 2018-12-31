#ifndef MULTI_MODBUS_SLAVE_H
#define MULTI_MODBUS_SLAVE_H

// SimpleModbusSlaveV10

/*
 SimpleModbusSlave allows you to communicate
 to any slave using the Modbus RTU protocol.
 
 This implementation DOES NOT fully comply with the Modbus specifications.
 
 Specifically the frame time out have not been implemented according
 to Modbus standards. The code does however combine the check for
 inter character time out and frame time out by incorporating a maximum
 time out allowable when reading from the message stream.
 
 SimpleModbusSlave implements an unsigned int return value on a call to modbus_update().
 This value is the total error count since the slave started. It's useful for fault finding.
 
 This code is for a Modbus slave implementing functions 3, 6 and 16
 function 3: Reads the binary contents of holding registers (4X references)
 function 6: Presets a value into a single holding register (4X reference)
 function 16: Presets values into a sequence of holding registers (4X references)
 
 All the functions share the same register array.
 
 Note:  
 The Arduino serial ring buffer changed in V1.05 it is now 64 bytes or 32 unsigned int registers.
 Most of the time you will connect the arduino to a master via serial
 using a MAX485 or similar.
 
 In a function 3 request the master will attempt to read from your
 slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
 and two BYTES CRC the master can only request 58 bytes or 29 registers.
 
 In a function 16 request the master will attempt to write to your 
 slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
 NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
 54 bytes or 27 registers.
 
 Using a USB to Serial converter the maximum bytes you can send is 
 limited to its internal buffer which differs between manufactures. 
 
 The functions included here have been derived from the 
 Modbus Specifications and Implementation Guides
 
 http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
 http://www.modbus.org/docs/PI_MBUS_300.pdf
 
*/

#include "Arduino.h"
#define SlaveBUFFER_SIZE 64

class MultiModbusSlave
{

    public:
        // function definitions
        unsigned int modbus_slaveupdate();
        void modbus_update_comms(long baud, unsigned char _slaveID,unsigned char _slaveID2);
        void modbus_slaveconfigure(HardwareSerial *SerialPort,
                                                    long baud,
                                                    unsigned char _slaveID,unsigned char _slaveID2, 
                            unsigned char _TxEnablePin, 
                            unsigned int _holdingRegsSize,
                            unsigned int* _regs,unsigned int* _regs2);
        unsigned int SlaveerrorCount;
    private:
        
        // Slavefunction definitions
        void exceptionResponse(unsigned char exception);
        unsigned int SlavecalculateCRC(unsigned char bufferSize); 
        void SlavesendPacket(unsigned char bufferSize);

        // Slaveframe[] is used to recieve and transmit packages. 
        // Variables 
        unsigned char Slaveframe[SlaveBUFFER_SIZE];
        unsigned int holdingRegsSize; // size of the register array
        unsigned int* Sregs;
        unsigned int* Sregs2; // user array address
        unsigned char SlavebroadcastFlag;
        unsigned char slaveID;
        unsigned char slaveID2;
        unsigned char Slavefunction;
        unsigned char SlaveTxEnablePin;
       // unsigned int SlaveerrorCount;
        unsigned int SlaveT1_5; // inter character time out
        unsigned int T3_5; // Slaveframe delay
};

#endif
