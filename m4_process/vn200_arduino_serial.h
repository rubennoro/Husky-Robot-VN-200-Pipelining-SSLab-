// Copyright (c) 2018 Flight Dynamics and Control Lab

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


// NOTE: Before running this code, you must configure the IMU as
// defined in the README file. If you want to get different data,
// you must update the respective parameters while configuring the
// IMU. 
// Further, this assumes you are using an Arduino board which has
// more than one serial ports. Serial1 is connected to the IMU and
// Serial is connected to a computer through a USB cable. If you do
// not wish to visualize data in the serial monitor, you may use
// Serial as the IMU port.
#include <RPC.h>
#include "share_memory_m4.h"

// Function declarations
void read_vn200(void); // while loop task
void read_imu_data(void);
bool check_imu_sync_byte(void);
unsigned short calculate_imu_crc(byte data[], unsigned int length);


// Union functions for byte to float conversions
// IMU sends data as bytes, the union functions are used to convert
// these data into other data types

// Attitude data
union {float f; byte b[4];} yaw;
union {float f; byte b[4];} pitch;
union {float f; byte b[4];} roll;

// Acceleration
union {float f; byte b[4];} a_x;
union {float f; byte b[4];} a_y;
union {float f; byte b[4];} a_z;

// Angular rates
union {float f; byte b[4];} W_x;  
union {float f; byte b[4];} W_y;  
union {float f; byte b[4];} W_z;  

// Checksum
union {unsigned short s; byte b[2];} checksum;


// Parameters
byte serial1Buffer[100];  // array to save data send from the IMU
byte calculateCRC(byte data[], unsigned int length);

long int imu_time;

// void setup() {

//   // Start Serial for printing data to the Serial Monitor
  
//   Serial.begin(2000000);

//   // Start Serial1 for IMU communication
//   Serial1.begin(921600);

//   while(!Serial);
//   while(!Serial1);
//   Serial.println("start: ");

//   // delay(3000);
//   // while(1){
//   //   // Serial.println(Serial1.readStringUntil('\n'));
//   //   int bufferSize = 44;
//   //   byte buffer[bufferSize];
//   //   Serial1.readBytes(buffer, bufferSize);
//   //   for(int i=0; i<bufferSize;i++){
//   //     Serial.print(buffer[i],HEX);
//   //     Serial.print(" ");
//   //   }
//   //     Serial.println();

//   // }
//   imu_time = micros();
// }


// void loop() {

//   // Check if new IMU data is available
//     read_imu();
  
// }
void sendIMU();

int size = 10;
void read_vn200(void){
    while (Serial1.available() < size) ;
    if(check_imu_sync_byte()){  // If sync byte is detected, read the rest of the data
      read_imu_data();
    }
  
}

// Check for the sync byte (0xFA)
bool check_imu_sync_byte(void) {
  for (int i = 0; i < size; i++) {
    Serial1.readBytes(serial1Buffer, 1);
    if (serial1Buffer[0] == 0xFA) {
      // imu_sync_detected = true;
      // Serial.print(in[0],HEX);
      // Serial.print(" ");
      return true;
    }
  }
  return false;
}

void read_imu_data(void) {
  Serial1.readBytes(serial1Buffer, 43);
  // for(int i=0; i<43;i++){
  //   Serial.print(in[i],HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // Serial.println(calculate_imu_crc(in, 43));
  // Serial.println(calculateCRC(in,  43));

  if (calculate_imu_crc(serial1Buffer, 43) == 0) {
    Serial.println("M4 data past the CRC, getting collected");
    for (int i = 0; i < 4; i++) {
      int j=1;
      j+=4;
      yaw.b[i] = serial1Buffer[j + i];
      j+=4;
      pitch.b[i] = serial1Buffer[j + i];
      j+=4;
      roll.b[i] = serial1Buffer[j + i];

      j+=4;
      a_x.b[i] = serial1Buffer[j + i];
      j+=4;
      a_y.b[i] = serial1Buffer[j + i];
      j+=4;
      a_z.b[i] = serial1Buffer[j + i];

      j+=4;
      W_x.b[i] = serial1Buffer[j + i];
      j+=4;
      W_y.b[i] = serial1Buffer[j + i];
      j+=4;
      W_z.b[i] = serial1Buffer[j + i];
    }

    // Serial.print(src);
    // Serial.print(" ");

    // Serial.print(yaw.f,2);
    // Serial.print(" ");
    // Serial.print(pitch.f,2);
    // Serial.print(" ");
    // Serial.print(roll.f,2);
    // Serial.print(" ");

    // RPC.print(a_x.f,4);
    // RPC.print(" ");

    // Serial.print(W_x.f);
    // Serial.print(" ");
    //  Serial.print(W_y.f);
    // Serial.print(" ");
    //  Serial.print(W_z.f);
    // Serial.print(" ");

    // receiveDataFromM4();

    sendIMU();

    // RPC.print("imu rate:  ");
    // RPC.println(1000000/(micros()-imu_time));
    // Serial.println("400");

    imu_time = micros();
  }

}

// Calculate the 16-bit CRC for the given ASCII or binary message.
unsigned short calculate_imu_crc(byte data[], unsigned int length)
{
  unsigned int i;
  unsigned short crc = 0;
  for(i=0; i<length; i++){
    crc = (byte)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    // Serial.print(data[i]);
    // Serial.print(" ");
    crc ^= (byte)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}
void sendIMU() {
    /* dummy data
    ii+=0.01;
    if(ii>32.76){
      ii=-32.76;
    }
    for(int i=0;i<9;i++){
      imu_values[i] = ii;
    }
    */

    imu_values[0] = a_x.f;
    imu_values[1] = a_y.f;
    imu_values[2] = a_z.f;

    imu_values[3] = W_x.f;
    imu_values[4] = W_y.f;
    imu_values[5] = W_z.f;

    imu_values[6] = yaw.f;
    imu_values[7] = pitch.f;
    imu_values[8] = roll.f;

    Serial.println(imu_values[8]);
    send_frame_ready_sdram->frame_ready = false;
    sendDataToM4();

}
// 44
// FA 05 08 00 00 06 7B 42 0F C3 AA AE 8D BF 59 99 1B 43 35 7E 66 BE 64 85 7D C0 BC B0 0D 41 5B 5C AA 3B 8B 54 AB BB B0 7A 4B 3A 8A 3A
// FA 05 08 00 00 06 2A 5C 2B C3 23 6D 86 40 B8 44 60 C1 72 52 3A 3F BC F7 18 40 74 AD 18 C1 96 6A 9E BA 41 4A 0D 3A 98 97 49 3A 32 04
// FA 05 08 00 00 06 C6 A4 00 43 82 BA 73 40 C4 1F F5 C1 9B 05 20 3F 51 FA A0 40 D6 74 07 C1 28 A2 F4 BA 00 8B E7 38 B0 E4 10 BA FB F5
