# MultiModbusSlave
Multiple Modbus Slave implementation on arduino based devices. Host multiple slaves on a single MCU device.
Now host multiple slave devices on a single mcu. Allows you to simultaneously act as master and slave.

For eg. Receive data as master from slave devices using one UART using simplemodbus master library and act as slave (using another RS485 module on software/hardware serial) to serve a master or act as Modbus RS485 gateway.


Based on SimpleModbusSlave  library by Juan Bester
Completed on 27th Aug'18 by Divyanshu Anand.
