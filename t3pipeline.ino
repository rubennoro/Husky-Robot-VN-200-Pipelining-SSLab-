/*
This program receives data from a source via serial communication. It reads in a 44 byte array, and processes
it into 11 four byte floats. 
*/
#define CUSTOM Cust
#include <SPI.h>
#include "t3pipeline.h"
#include "EasyCAT.h"

//Dual Core
#include <RPC.h>
#include "share_memory_m7.h"


//IMU DATA
static int16_t send_gx = 0;
static int16_t send_gy = 0;
static int16_t send_gz = 0;
static int16_t send_ax = 0;
static int16_t send_ay = 0;
static int16_t send_az = 0;
static int32_t send_yaw = 0;
static int32_t send_pitch = 0;
static int32_t send_roll = 0;


void rpc_print();
void send_convert_imu();
int16_t float_to_16int(float var);
int32_t float_to_32int(float var);

//bool hasReceivedDataFromM7 = False;


//Co-Processor Library
#include "vn200_arduino_serial.h"

#define MAX_INPUT 44
#define BYTES_PER 4
#define NUMS 11

EasyCAT EASYCAT;
byte array[MAX_INPUT];
float output[NUMS];

unsigned long start_time;
unsigned long second_start;

void setupEasyCAT();
void transfertoEasyCAT(float* output);
void reduceAccuracy(float& index);
void readBytes(byte* array, float* output);

long int main_time;
int flag = 1;
long int i2c_time;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(LED_BLUE, OUTPUT);
  while(!Serial){

  }
  
  setupEasyCAT();
  while(RPC.begin()!=1);
  Serial.println("Starting Dual Core");

  USERIAL = &Serial;

  Serial.println("Initialization Completed");

  main_time = micros();
}



void loop(){
  
  //Serial.println("Waiting for Data");
  //float time = micros() * 1000000; //change microseconds to a real value
  

  //This waits for the buffer to fill, do i need this?
  
  //THIS MAKES COMMS SYNCHRONOUS
  //while(Serial.available() < MAX_INPUT){};

  //second_start = micros();

  //Dual Core Process
  rpc_print();

  receiveDataFromM7(); //->automatically makes hasreceivedatafromm7 true, when data is available from m7, this is for synchronization of m4 data from imu and m7 data
  if(hasReceivedDataFromM7 == true){
    digitalWrite(LED_BLUE, LOW);
    send_convert_imu(); //about 200 microsec

    
    hasReceivedDataFromM7 = false;

    //IMU data transmission
    //Serial.print(" imu_rateM7:  ");
    //Serial.print(1000000/(micros()-imu_time));
    //imu_time = micros();
  }

  if(Serial.available() >= MAX_INPUT){
    flag=flag*-1;
  }
  if(flag==1){
      readBytes(array, output);
      flag = -1;

  }

  //SPI Transmission to EasyCAT Buffer
  transfertoEasyCAT(output);
  //EasyCAT transmission through EtherCAT to main 
  EASYCAT.MainTask();


  //Keeps time at 500 Hz
  if(micros()-main_time<1950){
    delayMicroseconds(2000-(micros()-main_time));
    //main_time = micros();
  }

  //float freq = 1000000/(micros() - main_time);
  //Serial.print("Rate: ");
  //Serial.println(freq);
  //main_time = micros();

  digitalWrite(LED_BLUE, HIGH);
}

void setupEasyCAT(){

    Serial.print ("\nEasyCAT - Generic EtherCAT slave\n");          // print the banner
                                                                    //---- initialize the EasyCAT board -----
                                                                    
    if (EASYCAT.Init() == true)                                     // initialization
    {                                                               // succesfully completed
      Serial.println("EASYCAT initialized");                                 //
    }                                                               //
    
    else                                                            // initialization failed   
    {                                                               // the EasyCAT board was not recognized
      Serial.print ("initialization failed");                       //     
                                                                    // The most common reason is that the SPI 
                                                                    // chip select choosen on the board doesn't 
                                                                    // match the one choosen by the firmware
                                                                    
                                                                    // with the Arduino led blinking
      while(1)                                                      //
      {                                                             //   
        digitalWrite (LED_BUILTIN, LOW);                                     // 
        delay(500);                                                //   
        digitalWrite (LED_BUILTIN, HIGH);                                    //  
        delay(500);                                                 // 
      }                                                             // 
    }  
  //preMicroTime = micros();
  //Serial.println(preMicroTime); //this was 103064
}

void readBytes(byte* array, float* output){
  Serial.readBytes(array, MAX_INPUT);
  for(int i = 0; i < NUMS; ++i) { //11 values
    int startIndex = i * BYTES_PER; //4 bytes per
    float* valuePtr = reinterpret_cast<float*>(&array[startIndex]); //4 byte increments casting

    output[i] = *valuePtr; // Access the `float` value from the byte array
    Serial.println(output[i]);
    //Function not working properly
    //reduceAccuracy(output[i]);

}
}
void reduceAccuracy(float& index){
  //Keep accuracy to six places
  int x = int(index * 1000000);
  index = x / 1000000.0;
}
//NEED CONVERSION / SYNC BYTE / CRC BYTES?

//Transmission of 11 Floats - 44 Bytes
void transfertoEasyCAT(float* output){
  int i = 0;  
	EASYCAT.BufferIn.Cust.bApp_gx = send_gx;
  EASYCAT.BufferIn.Cust.bApp_gy = send_gy;
  EASYCAT.BufferIn.Cust.bApp_gz = send_gz;
  EASYCAT.BufferIn.Cust.bApp_ax = send_ax;
  EASYCAT.BufferIn.Cust.bApp_ay = send_ay;
  EASYCAT.BufferIn.Cust.bApp_az = send_az;
  EASYCAT.BufferIn.Cust.bApp_yaw = send_yaw;
  EASYCAT.BufferIn.Cust.bApp_pitch = send_pitch;
  EASYCAT.BufferIn.Cust.bApp_roll = send_roll;

  EASYCAT.BufferIn.Cust.posx = output[i++];
  EASYCAT.BufferIn.Cust.posy = output[i++];
  EASYCAT.BufferIn.Cust.posz = output[i++];
  EASYCAT.BufferIn.Cust.q1 = output[i++];
  EASYCAT.BufferIn.Cust.q2 = output[i++];
  EASYCAT.BufferIn.Cust.q3 = output[i++];
  EASYCAT.BufferIn.Cust.q4 = output[i++];
  EASYCAT.BufferIn.Cust.rs1 = output[i++];
  EASYCAT.BufferIn.Cust.rs2 = output[i++];
  EASYCAT.BufferIn.Cust.rs3 = output[i++];
  EASYCAT.BufferIn.Cust.rs4 = output[i];


}

void rpc_print(){
  String buffer = "";
  while (RPC.available()) {
    buffer += (char)RPC.read();  // Fill the buffer with characters
  }
  if (buffer.length() > 0) {
    Serial.println(buffer);
  }
}

//Converstaion for the Linear Accel / Angular Velo
int16_t float_to_16int(float var)
{
	// data range: +-32.000, resolution: 0.001
	float temp = var;
	if (temp > 32) temp = 32;
	else if (temp < -32) temp = -32;
//	int16_t value = (int16_t) (temp*1000.0);

	return (int16_t) (temp * 1000);
}


//Conversion for the Yaw, Pitch, Roll
int32_t float_to_32int(float var)
{
	// data range: +-32.000, resolution: 0.001
	float temp = var;
	// if (temp > 32) temp = 32;
	// else if (temp < -32) temp = -32;
//	int16_t value = (int16_t) (temp*1000.0);

	return (int32_t) (temp * 1000);
}

void send_convert_imu(){

  send_ax = float_to_16int(imu_data_recv.ax);
  send_ay = float_to_16int(imu_data_recv.ay);
  send_az = float_to_16int(imu_data_recv.az);
  
  //Serial.println(send_ax); //check if data is accurate and available
  //Serial.println(send_ay);
  //Serial.println(send_az);

  send_gx = float_to_16int(imu_data_recv.gx); 
  send_gy = float_to_16int(imu_data_recv.gy); 
  send_gz = float_to_16int(imu_data_recv.gz); 

  //Serial.println(send_gx);// check data
  //Serial.println(send_gy);
  //Serial.println(send_gz);

  send_yaw = float_to_32int(imu_data_recv.mx);
  send_pitch = float_to_32int(imu_data_recv.my);
  send_roll = float_to_32int(imu_data_recv.mz);
  //Serial.println(send_yaw); //check data
  //Serial.println(send_pitch); //check data
  //Serial.println(send_roll); //check data

}

