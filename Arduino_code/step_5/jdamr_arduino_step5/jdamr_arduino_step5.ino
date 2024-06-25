// Install following library 
//https://github.com/hideakitai/MPU9250 

#include "MPU9250.h"
#include <SoftwareSerial.h>

// Step2 command protocol constant 
#define HEAD              0xf5
#define CMD_SET_MOTOR     0x01
#define CMD_GET_SPEED     0x02
#define CMD_GET_ENCODER   0x03
#define CMD_CAR_RUN       0x04
#define CMD_GET_IMU       0x05

//SoftwareSerial Serial2(7,8);

MPU9250 mpu;

void byte_to_hex(byte a){
  char hexString[3];
  sprintf(hexString, "%02X", a);
  // Serial2.println(hexString);  // for debugging 
}

void setup() {
    Serial.begin(115200);
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial2.println("hello...");
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial2.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
}

float pitch = 0, roll = 0, yaw = 0;
long pitch_l = 0, roll_l = 0, yaw_l = 0;

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 50) {

            byte data_buffer[16] = {0};   // Command packer container 
            byte i, checknum = 0;
            
            pitch = mpu.getPitch();
            roll = mpu.getRoll();
            yaw = mpu.getYaw();

            pitch_l = long(pitch);
            roll_l = 4567; //(long)(roll);
            yaw_l = 7890; //(long)(yaw);
            

            data_buffer[0] = 0xf5;                          // HEADER 
            data_buffer[1] = 15;                            // length 
            data_buffer[2] = CMD_GET_IMU;                   // command 
            data_buffer[3] = (pitch_l >> 24) & 0xff;     // payload 
            data_buffer[4] = (pitch_l >> 16) & 0xff; 
            data_buffer[5] = (pitch_l >> 8) & 0xff;
            data_buffer[6] = pitch_l & 0xff;
            data_buffer[7] = (roll_l >> 24) & 0xff;     // payload 
            data_buffer[8] = (roll_l >> 16) & 0xff; 
            data_buffer[9] = (roll_l >> 8) & 0xff;
            data_buffer[10] = roll_l  & 0xff;
            data_buffer[11] = (yaw_l >> 24) & 0xff;     // payload 
            data_buffer[12] = (yaw_l >> 16) & 0xff; 
            data_buffer[13] = (yaw_l >> 8) & 0xff;
            data_buffer[14] = yaw_l  & 0xff;
            for (i = 2; i < 15; i++)
            {
                checknum += data_buffer[i];
            }
            data_buffer[15] = checknum;           
            Serial.write(data_buffer, 10);  // Sending command packets 

            //print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
}
