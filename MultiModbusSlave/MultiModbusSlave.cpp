// Updated on 31/12/2018
// fixed major bugs include network RS485 incomptibility issues
// Updated intercharacter delay and interframe delay as per standard modbus specs of 11 char instead of 10 chars
#include "MultiModbusSlave.h"
#include "HardwareSerial.h"

// based on SimpleModbusSlaveV10

// Slaveframe[] is used to recieve and transmit packages. 
HardwareSerial* ModbusSlavePort;

void MultiModbusSlave::modbus_slaveconfigure(HardwareSerial *SerialPort,
                                                    long baud,
                                                    unsigned char _slaveID,unsigned char _slaveID2, 
                            unsigned char _TxEnablePin, 
                            unsigned int _holdingRegsSize,
                            unsigned int* _regs,unsigned int* _regs2)
    
{
  delay(0);
  ModbusSlavePort = SerialPort;
  modbus_update_comms(baud, _slaveID, _slaveID2);
  holdingRegsSize = _holdingRegsSize; 
  Sregs = _regs;
  Sregs2= _regs2;
  SlaveTxEnablePin = _TxEnablePin; 
  pinMode(SlaveTxEnablePin, OUTPUT);
  digitalWrite(SlaveTxEnablePin, LOW);
  SlaveerrorCount = 0; // initialize SlaveerrorCount
} 

void MultiModbusSlave::modbus_update_comms(long baud, unsigned char _slaveID, unsigned char _slaveID2)
{
	(*ModbusSlavePort).begin(baud, SERIAL_8N1);
	slaveID = _slaveID;
    slaveID2 = _slaveID2;
	
	// Modbus states that a baud rate higher than 19200 must use a fixed 750 us 
  // for inter character time out and 1.75 ms for a Slaveframe delay for baud rates
  // below 19200 the timing is more critical and has to be calculated.
  // E.g. 9600 baud in a 10 bit packet is 960 characters per second
  // In milliseconds this will be 960characters per 1000ms. So for 1 character
  // 1000ms/960characters is 1.04167ms per character and finally modbus states
  // an inter-character must be 1.5T or 1.5 times longer than a character. Thus
  // 1.5T = 1.04167ms * 1.5 = 1.5625ms. A Slaveframe delay is 3.5T.
	
	if (baud > 19200)
	{
		SlaveT1_5 = 750; 
		T3_5 = 1750; 
	}
	else 
	{ 
		// 11 characters is the modbus standard for a packet sizze , adjust interchar and frame delay accordingly
		//SlaveT1_5 = 15000000/baud; // 1T * 1.5 = T1.5
		SlaveT1_5 = 16500000/baud;
		//T3_5 = 35000000/baud; // 1T * 3.5 = T3.5//
		T3_5 = 38500000/baud;
		

	}
}  

unsigned int MultiModbusSlave::modbus_slaveupdate()
{
	
  if ((*ModbusSlavePort).available())
  {
	  unsigned char Sbuffer = 0;
	  unsigned char overflow = 0;
	
	  while ((*ModbusSlavePort).available())
	  {
		  // If more bytes is received than the SlaveBUFFER_SIZE the overflow flag will be set and the 
		  // serial buffer will be red untill all the data is cleared from the receive buffer.
		  delay(0);
		  if (overflow) 
			  (*ModbusSlavePort).read();
		  else
		  {
			  if (Sbuffer == SlaveBUFFER_SIZE)
				  overflow = 1;
			  Slaveframe[Sbuffer] = (*ModbusSlavePort).read();
			  Sbuffer++;
		  }
		  delayMicroseconds(SlaveT1_5); // inter character time out
	  }
	
	  // If an overflow occurred increment the SlaveerrorCount
	  // variable and return to the main sketch without 
	  // responding to the request i.e. force a timeout
	  if (overflow)
	  	{	//Serial.println("Overflow in reading buffer");
			  return SlaveerrorCount++;
		  }
	
	  // The minimum request packet is 8 bytes for Slavefunction 3 & 16
    if (Sbuffer > 7) 
	  {
		  unsigned char id = Slaveframe[0];
		
		  SlavebroadcastFlag = 0;
		
		  if (id == 0)
			  SlavebroadcastFlag = 1;
		
      if (id == slaveID || id == slaveID2 || SlavebroadcastFlag) // if the recieved ID matches the slaveID or broadcasting id (0), continue
      {
        unsigned int crc = ((Slaveframe[Sbuffer - 2] << 8) | Slaveframe[Sbuffer - 1]); // combine the crc Low & High bytes
        if (SlavecalculateCRC(Sbuffer - 2) == crc) // if the calculated crc matches the recieved crc continue
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
							  unsigned char noOfBytes = no_of_registers * 2; 
                // ID, Slavefunction, noOfBytes, (dataLo + dataHi)*number of registers,
                //  crcLo, crcHi
							  unsigned char responseFrameSize = 5 + noOfBytes; 
							  if (id == slaveID)
                                {
                                    Slaveframe[0] = slaveID;
                                }
                                else if (id == slaveID2)
                                {
                                    Slaveframe[0] = slaveID2;
                                }
								else 
								{

								}
                              //Slaveframe[0] = slaveID;
							  Slaveframe[1] = Slavefunction;
							  Slaveframe[2] = noOfBytes;
							  address = 3; // PDU starts at the 4th byte
							  unsigned int temp;
								
							  for (index = startingAddress; index < maxData; index++)
						  	{     
                                  if (id == slaveID)
                                    {
                                    	temp = Sregs[index];
                                    }
                                    else if (id == slaveID2)
                                    {
                                    	temp = Sregs2[index];
                                    }
								  //temp = Sregs[index];
								 
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
							  exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
					  }
					  else
						  exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
				  }
					else if (Slavefunction == 6)
					{
						if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
						{
							unsigned char id = Slaveframe[0];
                            if (id == slaveID)
                            {
                                Sregs[startingAddress] = ((Slaveframe[4] << 8) | Slaveframe[5]);// the 4th and 5th elements in Slaveframe is the 16 bit data value
                            }
                            else if (id == slaveID2)
                            {
                                Sregs2[startingAddress] = ((Slaveframe[4] << 8) | Slaveframe[5]);// the 4th and 5th elements in Slaveframe is the 16 bit data value
                            }
                            //Sregs[startingAddress] = ((Slaveframe[4] << 8) | Slaveframe[5]);// the 4th and 5th elements in Slaveframe is the 16 bit data value
							
							// only the first 6 bytes are used for CRC calculation
							crc16 = SlavecalculateCRC(6); 
							Slaveframe[6] = crc16 >> 8; // split crc into 2 bytes
							Slaveframe[7] = crc16 & 0xFF;
								
							// a Slavefunction 16 response is an echo of the first 6 bytes from 
              // the request + 2 crc bytes
							if (!SlavebroadcastFlag) // don't respond if it's a broadcast message
							  SlavesendPacket(8);
						}
						else
							exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
					}
				  else if (Slavefunction == 16)
				  {
					  // Check if the recieved number of bytes matches the calculated bytes 
            // minus the request bytes.
					  // id + Slavefunction + (2 * address bytes) + (2 * no of register bytes) + 
            // byte count + (2 * CRC bytes) = 9 bytes
					  if (Slaveframe[6] == (Sbuffer - 9)) 
					  {
						  if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
						  {
							  if (maxData <= holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
							  {
								  address = 7; // start at the 8th byte in the Slaveframe
								
								  for (index = startingAddress; index < maxData; index++)
							  	  {
                                      unsigned char id = Slaveframe[0];
                                       if (id == slaveID)
                                        {
                                            
                                            Sregs[index] = ((Slaveframe[address] << 8) | Slaveframe[address + 1]);
                                            address += 2;
                                        }
                                        else if (id == slaveID2)
                                        {
                                            Sregs2[index] = ((Slaveframe[address] << 8) | Slaveframe[address + 1]);
                                            address += 2;
                                        }
									 // Sregs[index] = ((Slaveframe[address] << 8) | Slaveframe[address + 1]);
									 // address += 2;
								  }	
								
								  // only the first 6 bytes are used for CRC calculation
								  crc16 = SlavecalculateCRC(6); 
								  Slaveframe[6] = crc16 >> 8; // split crc into 2 bytes
								  Slaveframe[7] = crc16 & 0xFF;
								
								  // a Slavefunction 16 response is an echo of the first 6 bytes from 
                  // the request + 2 crc bytes
								  if (!SlavebroadcastFlag) // don't respond if it's a broadcast message
									  SlavesendPacket(8); 
							  }
							  else	
								  exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
						  }
						  else
							  exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
					  }
					  else 
						  SlaveerrorCount++; // corrupted packet
          }					
				  else
					  exceptionResponse(1); // exception 1 ILLEGAL Slavefunction
        }
			  else // checksum failed
				{
				//	SlaveerrorCount++;
				//	Serial.println("Failed checksum");
				}  
      } // incorrect id
    }
	  else if (Sbuffer > 0 && Sbuffer < 8)
			{
			//	SlaveerrorCount++; // corrupted packet
			//	Serial.println("Sbuffer > 0 && Sbuffer < 8");
			}
		  
  }
	return SlaveerrorCount;
}	

void MultiModbusSlave::exceptionResponse(unsigned char exception)
{
  // each call to exceptionResponse() will increment the SlaveerrorCount
	SlaveerrorCount++; 
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
       // Slaveframe[0] = slaveID;
		Slaveframe[1] = (Slavefunction | 0x80); // set MSB bit high, informs the master of an exception
		Slaveframe[2] = exception;
		unsigned int crc16 = SlavecalculateCRC(3); // ID, Slavefunction|0x80, exception code
		Slaveframe[3] = crc16 >> 8;
		Slaveframe[4] = crc16 & 0xFF;
    // exception response is always 5 bytes 
    // ID, Slavefunction + 0x80, exception code, 2 bytes crc
		SlavesendPacket(5); 
	}
}

unsigned int MultiModbusSlave::SlavecalculateCRC(unsigned char bufferSize) 
{
  unsigned int temp1, temp2, flag;
  temp1 = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp1 = temp1 ^ Slaveframe[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp1 & 0x0001;
      temp1 >>= 1;
      if (flag)
        temp1 ^= 0xA001;
    }
  }
  // Reverse byte order. 
  temp2 = temp1 >> 8;
  temp1 = (temp1 << 8) | temp2;
  temp1 &= 0xFFFF; 
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp1; 
}

void MultiModbusSlave::SlavesendPacket(unsigned char bufferSize)
{
  digitalWrite(SlaveTxEnablePin, HIGH);
		
  for (unsigned char i = 0; i < bufferSize; i++)
    (*ModbusSlavePort).write(Slaveframe[i]);
		
	(*ModbusSlavePort).flush();
	
	// allow a Slaveframe delay to indicate end of transmission
	delayMicroseconds(T3_5); 
	
  digitalWrite(SlaveTxEnablePin, LOW);
}