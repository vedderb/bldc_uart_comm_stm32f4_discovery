This is a test project for UART communication between the VESC and a stm32discovery board. Almost the full interface of the VESC is implemented in the example, only firmware upload is missing.  
  
Connect PB10 (TX) to RX on the VESC  
Connect PB11 (RX) to TX on the VESC  
Connect the grounds between the descovery board and the VESC.  

To upload the project to a discovery board, just connect the USB cable to the programmer USB port and type:  

make upload  

Connect the USB port of the discovery board to a computer and run a serial terminal such as screen. Type help to list available commands. Only a few commands are implemented, but more can be added easily in main.c.  
  
I have written a tutorial on how to use and port this code to other platforms here:  
http://vedder.se/2015/10/communicating-with-the-vesc-using-uart/

