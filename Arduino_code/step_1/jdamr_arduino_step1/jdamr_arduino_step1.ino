#include <SoftwareSerial.h>

// Using softserial as debugging. Arduino Uno has only 1 serial port. 
SoftwareSerial Serial2(7,8);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial2.println("hello...");
}

/* 
Encoder variable
We should use long data type. Arduino long data type is 4 bytes.
Arduino int data type is 2 bytes. 
*/
long lf_encoder = 0;        // left front encoder value 
long rf_encoder = 0;        // right fron encoder value 
long lr_encoder = 0;        // left rear encoder value 
long rr_encoder = 0;        // right rear encoder value

void loop(){ 
  // Encoder value simulation 
  lf_encoder += 100;
  Serial2.print( " LF:" );      // print for debugging 
  Serial2.print( lf_encoder );
  Serial2.print( " RF:" );
  Serial2.print( rf_encoder );
  Serial2.print( " LR:" );
  Serial2.print( lr_encoder );
  Serial2.print( " RR:" );
  Serial2.println( rr_encoder );
 
  byte data_buffer[20] = {0};   // Command packer container 
  byte i, checknum = 0;
  data_buffer[0] = 0xf5;                          // HEADER 
  data_buffer[1] = 19;                            // length 
  data_buffer[2] = 0x03;                          // command 
  data_buffer[3] = (lf_encoder >> 24) & 0xff;     // payload 
  data_buffer[4] = (lf_encoder >> 16) & 0xff;     // long 4 bytes as byte array 
  data_buffer[5] = (lf_encoder >> 8) & 0xff;
  data_buffer[6] = lf_encoder & 0xff;
  data_buffer[7] = (rf_encoder >> 24) & 0xff;
  data_buffer[8] = (rf_encoder >> 16) & 0xff;
  data_buffer[9] = (rf_encoder >> 8) & 0xff;
  data_buffer[10] = rf_encoder  & 0xff;
  data_buffer[11] = (lr_encoder >> 24) & 0xff;
  data_buffer[12] = (lr_encoder >> 16) & 0xff;
  data_buffer[13] = (lr_encoder >> 8) & 0xff;
  data_buffer[14] = lr_encoder  & 0xff;
  data_buffer[15] = (rr_encoder >> 24) & 0xff;
  data_buffer[16] = (rr_encoder >> 16) & 0xff;
  data_buffer[17] = (rr_encoder >> 8) & 0xff;
  data_buffer[18] = rr_encoder  & 0xff;
 for (i = 2; i < 19; i++)
  {
    checknum += data_buffer[i];
  }
  data_buffer[19] = checknum;           
  Serial.write(data_buffer, 20);  // Sending command packets 

  delay(1000);

}

