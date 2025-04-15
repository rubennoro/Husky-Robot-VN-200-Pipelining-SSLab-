#include "vn200_arduino_serial.h"
#include <RPC.h>
// #include "share_memory_m4.h"

void setup(){
  RPC.begin();
  USERIAL = &RPC;

  Serial1.begin(921600);

}

void loop(){
  // RPC.print("hello world");
  // RPC.println("hello world");
  read_vn200();
  // RPC.print(a_x.f,4);
  // RPC.print("ss ");
  // delay(1000);

}
