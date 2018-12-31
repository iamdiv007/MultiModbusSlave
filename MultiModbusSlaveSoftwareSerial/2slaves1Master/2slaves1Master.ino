/******************************************************************************************************************************************************
  
  --This Code shows hosting 2 slaves on a single arduino device with a single master on an Esp8266. 
  -- 2 RS485-ttl modules are connected.
    -- 1(for master) connected to HW UART  as R0-->Rx, DI-->Tx.
    --and other(for slave) to software serial on R0-->D1, DI-->D2. (R0 and DI are RS485- ttl module's pins)
    --D0 connects to DE,RE pin on slave Module and D3 connects to DE,RE for master Rs485 module.
  
  --Credits to developers of Libraries used below.
  --v2.0 written by Divyanshu Anand,completed on 20 Aug'18
*******************************************************************************************************************************************************/
///////////////////Library///////////////////////
#include <ESP8266WiFi.h>
#include <SimpleModbusMaster.h>
#include <SoftwareSerial.h>
#include <MultiModbusSlaveSoftwareSerial.h>


MultiModbusSlaveSS slave1;
//MultiModbusSlaveSS slave2;

/////////////////Hardware pin decleration///////////////
const int chipSelect = D8;    //chip select of SD card
#define TxEnablePin D3        // TX RX enable of rs485 to ttl converter

//////////////////// MODBUS Port information ///////////////////
#define baud 9600
#define timeout 5000
#define polling 200 // the scan rate
#define retry_count 50

//////////////////////Global variable decleration////////////
byte SlaveId1=1;
byte SlaveId2=2;
int indexAddr_A = 0;
int indexAddr_B = 1;
int Index_A = 0, Index_B = 0;
unsigned long currentMillis;
unsigned long previousMillis = 0;
unsigned long StartTime_A=0;
unsigned long EndTime_A=0;
unsigned long StartTime_B=0;
unsigned long EndTime_B=0;
float flow_A=0,flow_B=0;


int interval = 700, currentsizeA, currentsizeB;
String dataString_A, dataString_B, dataStringTwice, dataStringTwiceB;
double tot_A=0.0,tot_B=0.0;
double prevTot_A=0.0,prevTot_B=0.0;
int yy, mm, dd, hh, m, ss;
bool Active_A, Active_B;

#define HOLDING_REGS_SIZE 206
unsigned int holdingRegs[HOLDING_REGS_SIZE]; 
unsigned int holdingRegsB[HOLDING_REGS_SIZE]; 
#define TOTAL_NO_OF_REGISTERS 50      // The total amount of available memory on the master to store data
enum
{
  PACKET1,
  PACKET2,
  PACKET3,                          //**********SIDE A***********//
  PACKET4,
  PACKET5,
  PACKET6,
  PACKET7,
  PACKET8,                          
  PACKET9,
  PACKET10,
  PACKET11,
  PACKET12,
  PACKET13,
  PACKET14,
  PACKET15,
  PACKET16,
  PACKET17,
  PACKET18,
                                //********** SIDE B****************//
  PACKET19,
  PACKET20,
  PACKET21,
  PACKET22,
  PACKET23,
  PACKET24,
  PACKET25,
  PACKET26,
  PACKET27,
  PACKET28,
  PACKET29,
  PACKET30,
  PACKET31,
  PACKET32,
  PACKET33,
  PACKET34,
  PACKET35,
  PACKET36,
 
  
  TOTAL_NO_OF_PACKETS // leave this last entry
};

Packet packets[TOTAL_NO_OF_PACKETS]; // Create an array of Packets to be configured
unsigned int regs[TOTAL_NO_OF_REGISTERS];// Masters register array

////////////////////////////////////////SETUP////////////////////////////////////////////////////////////
void setup() 
{

 // pinMode(D0, OUTPUT);
 // digitalWrite(D0, LOW);

  Serial1.begin(9600);                                                                                              // D4 on node mcu for debug
  WiFi.disconnect() ;
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay(3000);                                                                                                    // wait for console opening


//Side A modbus_construct
  modbus_construct(&packets[PACKET1], SlaveId1, READ_HOLDING_REGISTERS, 100, 1, 1);// Status
  modbus_construct(&packets[PACKET2], SlaveId1, READ_HOLDING_REGISTERS, 101, 1, 2);// Unit price
  modbus_construct(&packets[PACKET3], SlaveId1, READ_HOLDING_REGISTERS, 102, 2, 3);// Current delivery Qty.
  modbus_construct(&packets[PACKET4], SlaveId1, READ_HOLDING_REGISTERS, 104, 2, 5);// Current delivery amount
  modbus_construct(&packets[PACKET5], SlaveId1, READ_HOLDING_REGISTERS, 106, 3, 7);// Totaliser
  modbus_construct(&packets[PACKET6], SlaveId1, READ_HOLDING_REGISTERS, 109, 3, 10);//total money
  modbus_construct(&packets[PACKET7], SlaveId1, READ_HOLDING_REGISTERS, 112, 1, 13);//pressure
  modbus_construct(&packets[PACKET8], SlaveId1, READ_HOLDING_REGISTERS, 113, 1, 14);//flow rate
  modbus_construct(&packets[PACKET9], SlaveId1, READ_HOLDING_REGISTERS, 114, 1, 15);//Diagnostics
  modbus_construct(&packets[PACKET10], SlaveId1, READ_HOLDING_REGISTERS, 115, 1, 16);//ambient temp
  modbus_construct(&packets[PACKET11], SlaveId1, READ_HOLDING_REGISTERS, 116, 1, 17);//gas temp
  modbus_construct(&packets[PACKET12], SlaveId1, READ_HOLDING_REGISTERS, 117, 2, 19);//total hose qty
  modbus_construct(&packets[PACKET13], SlaveId1, READ_HOLDING_REGISTERS, 200, 1, 20);//lock
  modbus_construct(&packets[PACKET14], SlaveId1, READ_HOLDING_REGISTERS, 201, 1, 21);//set unit price
  modbus_construct(&packets[PACKET15], SlaveId1, READ_HOLDING_REGISTERS, 202, 1, 22);//taget fill pressure
  modbus_construct(&packets[PACKET16], SlaveId1, READ_HOLDING_REGISTERS, 203, 1, 23);//max fill pressure
  modbus_construct(&packets[PACKET17], SlaveId1, READ_HOLDING_REGISTERS, 204, 1, 24);//seq parameter L-M
  modbus_construct(&packets[PACKET18], SlaveId1, READ_HOLDING_REGISTERS, 205, 1, 25);//seq parameter M-H

//Side B modbus_construct  
  modbus_construct(&packets[PACKET19], SlaveId2, READ_HOLDING_REGISTERS, 100, 1, 26);
  modbus_construct(&packets[PACKET20], SlaveId2, READ_HOLDING_REGISTERS, 101, 1, 27);
  modbus_construct(&packets[PACKET21], SlaveId2, READ_HOLDING_REGISTERS, 102, 2, 28);
  modbus_construct(&packets[PACKET22], SlaveId2, READ_HOLDING_REGISTERS, 104, 2, 30);
  modbus_construct(&packets[PACKET23], SlaveId2, READ_HOLDING_REGISTERS, 106, 3, 32);
  modbus_construct(&packets[PACKET24], SlaveId2, READ_HOLDING_REGISTERS, 109, 3, 35); 
  modbus_construct(&packets[PACKET25], SlaveId2, READ_HOLDING_REGISTERS, 112, 1, 36); 
  modbus_construct(&packets[PACKET26], SlaveId2, READ_HOLDING_REGISTERS, 113, 1, 37);
  modbus_construct(&packets[PACKET27], SlaveId2, READ_HOLDING_REGISTERS, 114, 1, 38);
  modbus_construct(&packets[PACKET28], SlaveId2, READ_HOLDING_REGISTERS, 115, 1, 39);
  modbus_construct(&packets[PACKET29], SlaveId2, READ_HOLDING_REGISTERS, 116, 1, 40);
  modbus_construct(&packets[PACKET30], SlaveId2, READ_HOLDING_REGISTERS, 117, 2, 42);
  modbus_construct(&packets[PACKET31], SlaveId2, READ_HOLDING_REGISTERS, 200, 1, 44);
  modbus_construct(&packets[PACKET32], SlaveId2, READ_HOLDING_REGISTERS, 201, 1, 45);
  modbus_construct(&packets[PACKET33], SlaveId2, READ_HOLDING_REGISTERS, 202, 1, 46);
  modbus_construct(&packets[PACKET34], SlaveId2, READ_HOLDING_REGISTERS, 203, 1, 47);
  modbus_construct(&packets[PACKET35], SlaveId2, READ_HOLDING_REGISTERS, 204, 1, 48);
  modbus_construct(&packets[PACKET36], SlaveId2, READ_HOLDING_REGISTERS, 205, 1, 49);

// config the master
  modbus_configure(&Serial, baud, SERIAL_8N1, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);   // Initialize the Modbus Finite State Machine
  // Configure the slave 
  // baud,slave id1, slave id2, enable pin, reg size
  slave1.modbus_slaveconfigure(9600,10,11, D0, HOLDING_REGS_SIZE);
  
  delay(500);
   
}


void loop() 
{
  //get data from the slave device
  modbus_update();
  // ready to host data as slave
  slave1.modbus_slaveupdate(holdingRegs,holdingRegsB);
  
  yield();
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {

    delay(0);
       
    if(!( bitRead(regs[1], 0)))
    { 
      dataString_A = ReadModbus_A(false);
    //  timestamp = GetTimeString();
      //Mark the dispenser filling stop time
      if(EndTime_A==0 && StartTime_A !=0)
      {
        EndTime_A = millis();
        Serial1.print("End time A: ");
        Serial1.println(EndTime_A);
        delay(0);
      }
                           
    }
    else 
    {
      if(StartTime_A==0)
      {
        StartTime_A = millis();
        Serial1.print("Start time A: ");
        Serial1.println(StartTime_A);
        delay(0);
      }
    } 

     if(!( bitRead(regs[26], 0)))
    { 
      dataString_B = ReadModbus_B(false);
  //    timestamp = GetTimeString();  
    // Mark the dispenser filing stop time
       if(EndTime_B==0 && StartTime_B !=0)
      {
        EndTime_B = millis();
        Serial1.print("End time B: ");
        Serial1.println(EndTime_B);
        delay(0);
      }                    
    }
    else 
    {
      if(StartTime_B==0)
      {
        StartTime_B = millis();
        Serial1.print("start time B: ");
        Serial1.println(StartTime_B);
        delay(0);
      }
    }   
  
    Active_A = bitRead(regs[1], 0);
    Active_B = bitRead(regs[26], 0);
    dataString_A="";
    dataString_B="";
  
    String hex1A= String(regs[7],HEX);
    
    String hex2A= String(regs[8],HEX);
    
    String hex3A= String(regs[9],HEX);
       
    String BCDnum_A = String(hex1A)+String(hex2A)+String(hex3A);
   
    tot_A = (double) BCDnum_A.toInt()/1000.000;
    
    flow_A = (float)regs[14]/100.00;
    dataString_A = ReadModbus_A(false);

    if ( Active_A == 0 && dataString_A != dataStringTwice && tot_A != prevTot_A  )//&& (unitprice_A * qty_A * price_A*flow_A) != 0  && !PrevStringA.equals(String(String(tot_A, 3) + "," + String(qty_A, 3))) ) //&& String((unitprice * qty_A), 2) == String(price_A, 2)
    {
        
        float duration_A = (float)(EndTime_A - StartTime_A)/1000.00;
        Serial1.print("Fill duration A: ");
        Serial1.println(duration_A);
        dataString_A = ReadModbus_A(true);
        dataString_A += String(duration_A); 
        Serial1.print(F("Data of A is: "));

        Serial1.println(dataString_A);
        dataStringTwice = dataString_A;
        EndTime_A = 0;
        StartTime_A = 0;
        prevTot_A = tot_A;        
        delay(0);    
      
    }

    delay(0);
    modbus_update();
    
  //  slave1.modbus_slaveupdate(holdingRegs);
    slave1.modbus_slaveupdate(holdingRegs,holdingRegsB);
    
    dataString_B = ReadModbus_B(false);
    String hex1B= String(regs[32],HEX);
    
    String hex2B= String(regs[33],HEX);
    
    String hex3B= String(regs[34],HEX);
       
    String BCDnum_B = String(hex1B)+String(hex2B)+String(hex3B);
   
    tot_B = (double)BCDnum_B.toInt()/1000.000;
    flow_B = (float)regs[37]/100.00;
  
    if ( Active_B == 0 && !dataString_B.equals(dataStringTwiceB) && tot_B != prevTot_B )//&& regs[13]*regs[17] !=0 )//&& (unitprice_B * qty_B * price_B*flow_B) != 0 && !PrevStringB.equals(String(String(tot_B, 3) + "," + String(qty_B, 3)))) // && String((unitprice_B * qty_B), 2) == String(price_B, 2)
    {
  
        float duration_B = (float)(EndTime_B - StartTime_B)/1000.00;
        Serial1.print("Fill duration B: ");
        Serial1.println(duration_B);
        dataString_B = ReadModbus_B(true);
        dataString_B += String(duration_B); 

        Serial1.println(F("Side B's Data"));
//        Serial1.print(timestamp);
        Serial1.println(dataString_B);
        dataStringTwiceB = dataString_B;
        EndTime_B = 0;
        StartTime_B = 0;
        prevTot_B = tot_B;
        delay(0);

    }
    previousMillis = currentMillis;
  } // Outside the scheduled block, wait for the program to go in timed interval block

//  slave2.modbus_slaveupdate(holdingRegsB);
// Update the reg values in the device to serve the master 
  for (int k=100;k<118;k++)
  {
    writeHoldingRegs(k,regs[k-99],1);
    delay(0);
  }
  for (int k=200;k<206;k++)
  {
    writeHoldingRegs(k,regs[k-180],1);
    delay(0);
  }
  for (int k=100;k<118;k++)
  {
    writeHoldingRegs(k,regs[k-74],2);
    delay(0);
  }
  for (int k=200;k<206;k++)
  {
    writeHoldingRegs(k,regs[k-156],2);
    delay(0);
  }
  // Reenable the modbus connection if it fails
    if ( packets->connection == 0)
      {
        Serial1.println(F("Reenable  MODBUS communication "));
        Serial1.println(F(""));
        packets->connection = true;   
      }
    

}


// Function to return all the read modbus data in string
String ReadModbus_A(bool debug)
{
  float price = (float)regs[2]/100.00;
  
  //Read Qty as 32 bit data stored in 2 regs
  uint32_t qty_int = (uint32_t)regs[4]<<16|regs[3];
  float qty = (float) qty_int/1000.000;
  
  //Read Amount as 32 bit data stored in 2 regs
  uint32_t amt_int = (uint32_t)regs[6]<<16|regs[5];
  double amt = (double)amt_int/100.00;
  
  String hex1= String(regs[7],HEX);
  
  String hex2= String(regs[8],HEX);
  
  String hex3= String(regs[9],HEX);
     
  String BCDnum = String(hex1)+String(hex2)+String(hex3);
 
  double tot= (double) BCDnum.toInt()/1000.000;
  
  float pressure = (float)regs[13]/100.000;
  
  flow_A = (float)regs[14]/100.00;
  
  String output_stringA = String(tot,3)+","+String(qty,3)+","+String(amt,2)+","+String(price,2)+","+String(pressure,2)+","+String(flow_A,2)+",";
  delay(0);
  if(debug)
  {
    Serial1.print("Unit price: ");Serial1.println(price);
    Serial1.print("Amount: ");Serial1.println(amt);
    Serial1.print("Totaliser: ");
    Serial1.println(tot,3);
    Serial1.print("Pressure (bar): ");Serial1.println(pressure);
    Serial1.print("Flow Rate (kg/min): ");Serial1.println(flow_A);    
  }
  return output_stringA;
  
}

String ReadModbus_B(bool debug)
{
  float price = (float)regs[27]/100.00;
 
  //Read Qty as 32 bit data stored in 2 regs
  uint32_t qty_int = (uint32_t)regs[29]<<16|regs[28];
  float qty = qty_int/1000.000;
  
  //Read Amount as 32 bit data stored in 2 regs
  uint32_t amt_int = (uint32_t)regs[31]<<16|regs[30];
  float amt = (float)amt_int/100.00;
  
  String hex1= String(regs[32],HEX);
  
  String hex2= String(regs[33],HEX);
  
  String hex3= String(regs[34],HEX);
     
  String BCDnum = String(hex1)+String(hex2)+String(hex3);
 
  double tot= (double)BCDnum.toInt()/1000.000;
  
  float pressure = (float)regs[36]/100.00;
  
  flow_B = (float)regs[37]/100.00;
  
  String output_stringB = String(tot,3)+","+String(qty,3)+","+String(amt,2)+","+String(price,2)+","+String(pressure,2)+","+String(flow_B,2)+",";                      
  delay(0);
  if (debug)
  {
    Serial1.print("Unit price: ");Serial1.println(price);
    Serial1.print("Amount: ");Serial1.println(amt);
    Serial1.print("Totaliser: ");
    Serial1.println(tot,3);
    Serial1.print("Pressure (bar): ");Serial1.println(pressure);
    Serial1.print("Flow Rate (kg/min): ");Serial1.println(flow_B);
  }
  return output_stringB;  
}


void writeHoldingRegs(int index, uint16_t regValue, int side)
{
  if(side==1)
  {
    holdingRegs[index] = regValue;
    delayMicroseconds(50);
  }
  else if(side==2)
  {
    holdingRegsB[index] = regValue;
    delayMicroseconds(50);
  }
  delay(0);
}
