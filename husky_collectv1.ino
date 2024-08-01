#define CUSTOM Cust
#include "husky_collectv1.h"
#include "EasyCAT.h"
#include <SPI.h>

#define MAX_INPUT 41
#define SIZE 19 // Adjusted to match the number of data points

//Co-Processor Libraries
#include "vn200_arduino_serial.h"

//Duo core
#include <RPC.h>
#include "share_memory_m7.h"

/*
// Variables for 24 data points
uint16_t position1x, position2x, position3x, position4x, position5x;
uint16_t position1y, position2y, position3y, position4y, position5y;
uint16_t position1z, position2z, position3z, position4z, position5z;
uint16_t yaw, pitch, row, gyrox, gyroy, gyroz;
uint16_t other1, other2, other3;
*/
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

EasyCAT EASYCAT;

//Size of data 
//const int size = 19;

//Two Arrays used
byte input[41];

//convert data initially to float
float initial_collection[19];

//convert data from float to uint16_t
uint16_t intData[19];

int delimiterCount = 0;
bool readingNumber = false;

// Run Time variables for testing
unsigned long main_time;
unsigned long loop_start_time;
unsigned long collect_time;
unsigned long conversion_time;
unsigned long toeasycatime;
unsigned long tocomptime;


//uint16_t float_to_uint(float var);
int16_t float_to_int(float var);
bool check_sync_byte(byte* input);
void processData(byte* input, float* output);
float bytes2float_02(byte b1, byte b2);
unsigned short calculate_m7_crc(byte* data, unsigned int length);
void transfer_to_EasyCAT(uint16_t* output);
void setupEasyCAT();
void arrayConversion(float* floatData, uint16_t* intData);

void rpc_print();
void send_convert_imu();
int32_t float_to_32int(float var);
//bool hasReceivedDataFromM7 = false;

void setup(){

  Serial.begin(921600);
  while(!Serial);

  setupEasyCAT();

  //Serial.println("Initialization Complete.");
  pinMode(LED_BUILTIN, OUTPUT); //used for tests

//  delay(10000); ->used to give time for manual input to the serial monitor for run time testing
  //main_time = micros();
  //loop_start_time = micros();

  //initialize the rpc communication
  while(RPC.begin()!=1);
  Serial.println("Duo core start:");

  USERIAL = &Serial;
  
}

void loop(){

  /*
  1. Ensure data is in the buffer and can be used
  2. RPC_print the m4 data
  3. ensure data was received by the m7
  4. convert the imu data once m7 data is received for synchronization
  5. read the byte array data
  6. run the spi and ethercat protocol to transfer data to computer

  */

  //Ensure there are at least 41 bytes in the serial port to read, otherwise there is not enough information for another cycle
  Serial.println("Waiting for Data");
  while(Serial.available() < MAX_INPUT){};
  Serial.println("Data Received");
  //delay(2000);

  rpc_print();

  loop_start_time = micros(); // Record the start time of the process after the data is available
  
  receiveDataFromM7(); //->automatically makes hasreceivedatafromm7 true, when data is available from m7, this is for synchronization of m4 data from imu and m7 data
  if(hasReceivedDataFromM7 == true){
    send_convert_imu(); //about 200 microsec
    hasReceivedDataFromM7 = false;
  }

  imu_time = micros();

  if(check_sync_byte(input)){
    processData(input, initial_collection); //internal CRC check, about 200 microsec
  }
  else{
    Serial.println("Incorrect sync byte, wrong data.");
    while(1)                                                      //
      {                                                             //   
        //digitalWrite (LED_BUILTIN, LOW);                                     // 
        //delay(500);                                                //   
        //digitalWrite (LED_BUILTIN, HIGH);                                    //                                                   // 
      }       
  }
  collect_time = micros();// ->TEST
  
  //Convert array to proper datatype
  arrayConversion(initial_collection, intData); //2 microsec

  conversion_time = micros();// ->TEST

  Serial.println("Transferring Data to EasyCAT");
  
  //SPI CONVERSION about 200 microsec
  transfer_to_EasyCAT(intData); //NEED TO INCLUDE IMU DATA POINTS HERE

  toeasycatime = micros();// ->TEST
  //Serial.println("Check for red light by this point");
  //delay(1000);
  Serial.println("Completing data transfer to computer");
  EASYCAT.MainTask(); //NEED TO INCLUDE IMU DATA HERE, about 600 microsec

  tocomptime = micros(); //->TEST
  
  //FREQUENCY TESTING
  unsigned long loop_duration = micros() - loop_start_time;
  float frequency = 1000000.0 / loop_duration; // Frequency in Hz
  Serial.print("Loop frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");
  Serial.print("IMU Time: ");
  Serial.println(imu_time-loop_start_time);
  Serial.print("Data processing time: ");
  Serial.println(collect_time - imu_time);
  Serial.print("Conversion time: ");
  Serial.println(conversion_time - collect_time);
  Serial.print("EasyCAT time: ");
  Serial.println(toeasycatime - conversion_time);
  Serial.print("Computer time: ");
  Serial.println(tocomptime - toeasycatime);
   

}

//EasyCAT setup function
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

/*
//OLD FUNCTION
//Find_data function takes in an array of floats size 24, which is empty, and parses through data to find each number and save it
void find_data(float* readData){

  //Set this back to 0 for each new set of 24 data points
  delimiterCount = 0;

  while (Serial.available() > 0) {
    //Save the character
    char add = Serial.read();

    //Condition for if it is a digit or a decimal
    if (isdigit(add) || add == '.') {
      //Boolean value used to prevent initial case where the 
      readingNumber = true;
      //Save all data into a string that is reset each time
      String temp = "";
      while (isdigit(add) || add == '.') {
        //Save all data into a string
        temp += add;
        if(Serial.available() > 0){
          add = Serial.read();
        }
      }
      initial_collection[delimiterCount] = temp.toFloat();
      //Serial.println(temp);
    }

    //Increments delimiterCount after prescence of a delimiter stops the loop above
    if (!isdigit(add) && readingNumber) {
      readingNumber = false;
      delimiterCount++;
    }

    //When there are 24 data points from 0 to 23, the delimiter count will be 24. When this is the case, need to change 
    //the floats to u_ints
    if (delimiterCount == SIZE) {
      break;
    }
  }

}*/

bool check_sync_byte(byte* input){
  for(int i = 0; i < MAX_INPUT; i++){
    Serial.readBytes(input, 1);
    if(input[0] == 0x2A){
      return true;
    }
  }
  //Serial.println("FALSE");
  return false;
}

void processData(byte* input, float* output){

  Serial.readBytes(input, 40);
  
  //add if statment for checksum verification
  //if(calculate_m7_crc(input, 40)==0){
    Serial.println("CRC Check Successful");
    for(int i = 0; i < SIZE; i++){
    //digitalWrite (LED_BUILTIN, LOW);                    
    //delay(500);
    //Serial.println("If code ends here, there is error with processing data");
    output[i] = bytes2float_02(input[i*2+1], input[i*2+2]);
    //delay(1000);
    //digitalWrite (LED_BUILTIN, HIGH);  
  }
  }
  /*else{
    Serial.println("CRC Check Failed");
  }*/
//}
/*
//DEPENDS ON THE INITIAL ALGORITHM
unsigned short calculate_m7_crc(byte* data, unsigned int length)
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
}*/

float bytes2float_02(byte b1, byte b2){
	float output = 0;
	long int i1 = (int)(b1) - 1;
	long int i2 = (int)(b2) - 1;
	long int value = i1 * 255 + i2; // uint8_t
	output = (float)(value - 32512) / 10000.0 *2;
  //Serial.println(output);
	return output;
}

//Converts data to an array of uint_16's
void arrayConversion(float* floatData, uint16_t* intData){
  for(int i = 0; i < SIZE; i++){

    intData[i] = float_to_int(floatData[i]);
    //Serial.println("int16_t data");
   // Serial.println(intData[i]);
    //delay(1000);
    //Check with print statements
    //Serial.print("This is the uint16 data: ");
    //Serial.println(uintData[i]);
  }
}
/*
//Individually converts floats to uint16s
uint16_t float_to_uint(float var) {
  float temp = var / 1000;
  if (temp > 65.535) temp = 65.535;
  else if (temp < 0) temp = 0;
  return (uint16_t)(temp * 1000);
}*/

//Converts floats to int16's
int16_t float_to_int(float var)
{
	// data range: +-32.000, resolution: 0.001
	float temp = var;
	if (temp > 32) temp = 32;
	else if (temp < -32) temp = -32;
//	int16_t value = (int16_t) (temp*1000.0);

	return (int16_t) (temp * 1000);
}


//Transfers the data from EasyCAT to Computer
void transfer_to_EasyCAT(uint16_t* output) { //collection of data to computer
  int i = 0;
  
  EASYCAT.BufferIn.Cust.x = output[i++];
  EASYCAT.BufferIn.Cust.y = output[i++];
  EASYCAT.BufferIn.Cust.z = output[i++];
  EASYCAT.BufferIn.Cust.q1 = output[i++];
  EASYCAT.BufferIn.Cust.q2 = output[i++];
  EASYCAT.BufferIn.Cust.q3 = output[i++];
  EASYCAT.BufferIn.Cust.q4 = output[i++];
  EASYCAT.BufferIn.Cust.x1 = output[i++];
  EASYCAT.BufferIn.Cust.y1 = output[i++];
  EASYCAT.BufferIn.Cust.z1 = output[i++];
  EASYCAT.BufferIn.Cust.x2 = output[i++];
  EASYCAT.BufferIn.Cust.y2 = output[i++];
  EASYCAT.BufferIn.Cust.z2 = output[i++];
  EASYCAT.BufferIn.Cust.x3 = output[i++];
  EASYCAT.BufferIn.Cust.y3 = output[i++];
  EASYCAT.BufferIn.Cust.z3 = output[i++];
  EASYCAT.BufferIn.Cust.x4 = output[i++];
  EASYCAT.BufferIn.Cust.y4 = output[i++];
  EASYCAT.BufferIn.Cust.z4 = output[i];

  /* IMU DATA
  EASYCAT.BufferIn.Cust.bApp_yaw = send_yaw;
  EASYCAT.BufferIn.Cust.bApp_pitch = send_pitch;
  EASYCAT.BufferIn.Cust.bApp_roll = send_roll;
	EASYCAT.BufferIn.Cust.bApp_gx = send_gx;
	EASYCAT.BufferIn.Cust.bApp_gy = send_gy;
	EASYCAT.BufferIn.Cust.bApp_gz = send_gz;
	EASYCAT.BufferIn.Cust.bApp_ax = send_ax;
	EASYCAT.BufferIn.Cust.bApp_ay = send_ay;
	EASYCAT.BufferIn.Cust.bApp_az = send_az;

  */
  

}

void rpc_print(){
  String buffer = "";
  while (RPC.available()) {
    buffer += (char)RPC.read();  // Fill the buffer with characters
    //Serial.println("Buffer characters reading");
  }
  if (buffer.length() > 0) {
    Serial.println(buffer);
  }
  else{
    Serial.println("No m4 print statements");
  }
}

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

  send_ax = float_to_int(imu_data_recv.ax);
  send_ay = float_to_int(imu_data_recv.ay);
  send_az = float_to_int(imu_data_recv.az);
  //Serial.println(send_ax); //check if data is accurate and available
  //Serial.println(send_ay);
  //Serial.println(send_az);

  send_gx = float_to_int(imu_data_recv.gx); 
  send_gy = float_to_int(imu_data_recv.gy); 
  send_gz = float_to_int(imu_data_recv.gz); 

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
