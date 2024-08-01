// #include "vn200_arduino_serial.h"

#include <arduino.h>
#include <RPC.h>
Stream *USERIAL = nullptr;

#include "SDRAM.h"

// Data constructors
struct IMU_DATA {
  float ax = 0.0;
  float ay = 0.0;
  float az = 0.0;
  float gx = 0.0;
  float gy = 0.0;
  float gz = 0.0;
  float mx = 0.0;
  float my = 0.0;
  float mz = 0.0;
} __attribute__((aligned(8)));

struct DATA_FRAME_SEND {
    IMU_DATA imu_data;
} __attribute__((aligned(8)));

struct DATA_FRAME_RETURN {
  char debugStringData[100];
} __attribute__((aligned(8)));

struct SEND_FRAME_READY_FLAG {
  volatile bool frame_ready = false;
} __attribute__((aligned(8)));

struct RETURN_FRAME_READY_FLAG {
  volatile bool frame_ready = false;
} __attribute__((aligned(8)));

//SDRAM Pointers
const uint32_t SDRAM_START_ADDRESS_4 = ((uint32_t)0x38000000);  //USING THE AHB SRAM4 DOMAIN SPACE
volatile uint32_t *sdramMemory = (uint32_t*)0x38000000;

SEND_FRAME_READY_FLAG* send_frame_ready_sdram = (SEND_FRAME_READY_FLAG*)(sdramMemory);
RETURN_FRAME_READY_FLAG* return_frame_ready_sdram = (RETURN_FRAME_READY_FLAG*)(sdramMemory + sizeof(SEND_FRAME_READY_FLAG) / sizeof(uint32_t));
DATA_FRAME_SEND* data_frame_send_sdram = (DATA_FRAME_SEND*)(sdramMemory + (sizeof(SEND_FRAME_READY_FLAG) + sizeof(RETURN_FRAME_READY_FLAG)) / sizeof(uint32_t));
DATA_FRAME_RETURN* data_frame_return_sdram = (DATA_FRAME_RETURN*)(sdramMemory + (sizeof(SEND_FRAME_READY_FLAG) + sizeof(RETURN_FRAME_READY_FLAG) + sizeof(DATA_FRAME_SEND)) / sizeof(uint32_t));


float imu_values[9];
float ii = 0;
void sendDataToM4();
void receiveDataFromM4();

void sendDataToM4() {
  if (!send_frame_ready_sdram->frame_ready) { //if false it can be written to
    //USERIAL->print(send_frame_ready_sdram->frame_ready);
    DATA_FRAME_SEND dataForM4;
    memcpy(&dataForM4.imu_data, &imu_values, sizeof(struct IMU_DATA));
    *data_frame_send_sdram = dataForM4;
    send_frame_ready_sdram->frame_ready = true; //if true M4 can read from it
    //USERIAL->println(send_frame_ready_sdram->frame_ready);
  }
}

void receiveDataFromM4() {
  if (return_frame_ready_sdram->frame_ready) {  //true indicates it can read by M7
    DATA_FRAME_RETURN dataFromM4 = *data_frame_return_sdram;
    //telemetry.printTelemetry(String(dataFromM4.debugStringData), TELEMETRY_LOW_PRIORITY);
    USERIAL->println(String(dataFromM4.debugStringData));
    return_frame_ready_sdram->frame_ready = false;  //false indicates data has been read by M7
  }
}

