/* Object oriented Library to host multiple modbus rtu slaves on a single device.
  Based on Simplemodbusslave Library by Juan Bester.
  v1.0 completed on 20/08/2018 by Divyanshu Anand

*/
#include "MultiModbusSlaveSoftwareSerial.h"
#include "SoftwareSerial.h"

#define BUFFER_SIZE 128


SoftwareSerial mySerial(D1, D2);

unsigned int MultiModbusSlaveSS::modbus_slaveupdate(unsigned int *holdingRegs,unsigned int *holdingRegs2)
{
  unsigned char buffer = 0;
  unsigned char overflow = 0;
  
  while (mySerial.available())
  {
    // The maximum number of bytes is limited to the serial buffer size of 128 bytes
    // If more bytes is received than the BUFFER_SIZE the overflow flag will be set and the 
    // serial buffer will be red untill all the data is cleared from the receive buffer.
    if (overflow) 
      mySerial.read();
    else
    {
      if (buffer == BUFFER_SIZE)
        overflow = 1;
      Slaveframe[buffer] = mySerial.read();
      buffer++;
    }
    delayMicroseconds(SlaveT1_5); // inter character time out
  }
  
  // If an overflow occurred increment the SlaveerrorCount
  // variable and return to the main sketch without 
  // responding to the request i.e. force a timeout
  if (overflow)
    return SlaveerrorCount++;
  
  // The minimum request packet is 8 bytes for Slavefunction 3 & 16
  if (buffer > 6) 
  {
    unsigned char id = Slaveframe[0];
    
    SlavebroadcastFlag = 0;
    
    if (id == 0)
      SlavebroadcastFlag = 1;
    
    if (id == slaveID || slaveID2 || SlavebroadcastFlag ) // if the recieved ID matches the slaveID or broadcasting id (0), continue
    {
      unsigned int crc = ((Slaveframe[buffer - 2] << 8) | Slaveframe[buffer - 1]); // combine the crc Low & High bytes
      if (SlavecalculateCRC(buffer - 2) == crc) // if the calculated crc matches the recieved crc continue
      {
        Slavefunction = Slaveframe[1];
        unsigned int startingAddress = ((Slaveframe[2] << 8) | Slaveframe[3]); // combine the starting address bytes
        unsigned int no_of_registers = ((Slaveframe[4] << 8) | Slaveframe[5]); // combine the number of register bytes  
        unsigned int maxData = startingAddress + no_of_registers;
        unsigned char index;
        unsigned char address;
        unsigned int crc16;
        
        // broadcasting is not supported for Slavefunction 3 
        if (!SlavebroadcastFlag && (Slavefunction == 3))
        {
          if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
          {
            if (maxData <= holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
            { 
              unsigned char id = Slaveframe[0];
              unsigned char noOfBytes = no_of_registers * 2;
              unsigned char responseFrameSize = 5 + noOfBytes; // ID, Slavefunction, noOfBytes, (dataLo + dataHi) * number of registers, crcLo, crcHi
              // check what device id is to be served
              if (id == slaveID)
              {
                Slaveframe[0] = slaveID;
              }
              else if (id == slaveID2)
              {
                Slaveframe[0] = slaveID2;
              }
              
              Slaveframe[1] = Slavefunction;
              Slaveframe[2] = noOfBytes;
              address = 3; // PDU starts at the 4th byte
              unsigned int temp;
              
              for (index = startingAddress; index < maxData; index++)
              {
                if (id == slaveID)
                {
                  temp = holdingRegs[index];
                }
                else if (id == slaveID2)
                {
                  temp = holdingRegs2[index];
                }
                //temp = holdingRegs[index];
                Slaveframe[address] = temp >> 8; // split the register into 2 bytes
                address++;
                Slaveframe[address] = temp & 0xFF;
                address++;
              } 
              
              crc16 = SlavecalculateCRC(responseFrameSize - 2);
              Slaveframe[responseFrameSize - 2] = crc16 >> 8; // split crc into 2 bytes
              Slaveframe[responseFrameSize - 1] = crc16 & 0xFF;
              SlavesendPacket(responseFrameSize);
            }
            else  
              SlaveexceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
          }
          else
            SlaveexceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
        }
        else if (Slavefunction == 6)
        {
          if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
          {
              unsigned int startingAddress = ((Slaveframe[2] << 8) | Slaveframe[3]);
              unsigned int regStatus = ((Slaveframe[4] << 8) | Slaveframe[5]);
              unsigned char responseFrameSize = 8;
              unsigned char id = Slaveframe[0];
              if (id == slaveID)
              {
                holdingRegs[startingAddress] = regStatus;
              }
              else if (id == slaveID2)
              {
                holdingRegs2[startingAddress] = regStatus;
              }
              //holdingRegs[startingAddress] = regStatus;
              
              crc16 = SlavecalculateCRC(responseFrameSize - 2);
              Slaveframe[responseFrameSize - 2] = crc16 >> 8; // split crc into 2 bytes
              Slaveframe[responseFrameSize - 1] = crc16 & 0xFF;
              SlavesendPacket(responseFrameSize);
          }
          else
            SlaveexceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
          }
        else if (Slavefunction == 16)
        {
          // check if the recieved number of bytes matches the calculated bytes minus the request bytes
          // id + Slavefunction + (2 * address bytes) + (2 * no of register bytes) + byte count + (2 * CRC bytes) = 9 bytes
          if (Slaveframe[6] == (buffer - 9)) 
          {
            if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
            {
              if (maxData <= holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
              {
                address = 7; // start at the 8th byte in the Slaveframe
                
                for (index = startingAddress; index < maxData; index++)
                {
                  if (id == slaveID)
                  {
                    unsigned char id = Slaveframe[0];
                    holdingRegs[index] = ((Slaveframe[address] << 8) | Slaveframe[address + 1]);
                    address += 2;
                  }
                  else if (id == slaveID2)
                  {
                    holdingRegs2[index] = ((Slaveframe[address] << 8) | Slaveframe[address + 1]);
                    address += 2;
                  }
                  //holdingRegs[index] = ((Slaveframe[address] << 8) | Slaveframe[address + 1]);
                 // address += 2;
                } 
                
                // only the first 6 bytes are used for CRC calculation
                crc16 = SlavecalculateCRC(6); 
                Slaveframe[6] = crc16 >> 8; // split crc into 2 bytes
                Slaveframe[7] = crc16 & 0xFF;
                
                // a Slavefunction 16 response is an echo of the first 6 bytes from the request + 2 crc bytes
                if (!SlavebroadcastFlag) // don't respond if it's a broadcast message
                  SlavesendPacket(8); 
              }
              else  
                SlaveexceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
            }
            else
              SlaveexceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
          }
          else 
            SlaveerrorCount++; // corrupted packet
        }         
        else
          SlaveexceptionResponse(1); // exception 1 ILLEGAL Slavefunction
      }
      else // checksum failed
        SlaveerrorCount++;
    } // incorrect id
  }
  else if (buffer > 0 && buffer < 8)
    SlaveerrorCount++; // corrupted packet
    
  return SlaveerrorCount;
}       

void MultiModbusSlaveSS::SlaveexceptionResponse(unsigned char exception)
{
  SlaveerrorCount++; // each call to SlaveexceptionResponse() will increment the SlaveerrorCount
  if (!SlavebroadcastFlag) // don't respond if its a broadcast message
  { 
    unsigned char id = Slaveframe[0];
    if (id == slaveID)
    {
      Slaveframe[0] = slaveID;
    }
    else if (id == slaveID2)
    {
      Slaveframe[0] = slaveID2;
    }
    
    Slaveframe[1] = (Slavefunction | 0x80); // set the MSB bit high, informs the master of an exception
    Slaveframe[2] = exception;
    unsigned int crc16 = SlavecalculateCRC(3); // ID, Slavefunction + 0x80, exception code == 3 bytes
    Slaveframe[3] = crc16 >> 8;
    Slaveframe[4] = crc16 & 0xFF;
    SlavesendPacket(5); // exception response is always 5 bytes ID, Slavefunction + 0x80, exception code, 2 bytes crc
  }
}

void MultiModbusSlaveSS::modbus_slaveconfigure(long baud, unsigned char _slaveID, unsigned char _slaveID2, unsigned char _TxEnablePin, unsigned int _holdingRegsSize)
{
  delay(0);
  slaveID = _slaveID;
  slaveID2 = _slaveID2;
  mySerial.begin(baud);
  
  if (_TxEnablePin > 1) 
  { // pin 0 & pin 1 are reserved for RX/TX. To disable set txenpin < 2
    SlaveTxEnablePin = _TxEnablePin; 
    pinMode(SlaveTxEnablePin, OUTPUT);
    digitalWrite(SlaveTxEnablePin, LOW);
  }
  
  // Modbus states that a baud rate higher than 19200 must use a fixed 750 us 
  // for inter character time out and 1.75 ms for a Slaveframe delay.
  // For baud rates below 19200 the timeing is more critical and has to be calculated.
  // E.g. 9600 baud in a 10 bit packet is 960 characters per second
  // In milliseconds this will be 960characters per 1000ms. So for 1 character
  // 1000ms/960characters is 1.04167ms per character and finaly modbus states an
  // intercharacter must be 1.5T or 1.5 times longer than a normal character and thus
  // 1.5T = 1.04167ms * 1.5 = 1.5625ms. A Slaveframe delay is 3.5T.
  
  if (baud > 19200)
  {
    SlaveT1_5 = 150; 
    T3_5 = 350; 
  }
  else 
  {
    SlaveT1_5 = 15000000/baud; // 1T * 1.5 = T1.5
    T3_5 = 35000000/baud; // 1T * 3.5 = T3.5
  }
  
  holdingRegsSize = _holdingRegsSize;
  SlaveerrorCount = 0; // initialize SlaveerrorCount
}   

unsigned int MultiModbusSlaveSS::SlavecalculateCRC(byte bufferSize) 
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp = temp ^ Slaveframe[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order. 
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  return temp; // the returned value is already swopped - crcLo byte is first & crcHi byte is last
}

void MultiModbusSlaveSS::SlavesendPacket(unsigned char bufferSize)
{
  if (SlaveTxEnablePin > 1)
    digitalWrite(SlaveTxEnablePin, HIGH);
    
  for (unsigned char i = 0; i < bufferSize; i++)
    mySerial.write(Slaveframe[i]);
    
  mySerial.flush();
  
  // allow a Slaveframe delay to indicate end of transmission
  delayMicroseconds(T3_5); 
  
  if (SlaveTxEnablePin > 1)
    digitalWrite(SlaveTxEnablePin, LOW);
}
