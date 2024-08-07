#include "mcp_can.h"
#include "TMotor_ServoConnection.h"

#define CAN0_INT 2  // CAN message pin
#define ID_MOTOR 1  // motor set with the CubeMars Upper Computer Software

MCP_CAN CAN0(10); // set SPI select pin to pin 10
TMotor_ServoConnection servo_conn(CAN0);  // create CAN Servo Connection

void setup() {
  // establisehd serial connection to PC
  while (!Serial);
  Serial.begin(115200);
  // establish connection between Arduino and MCP2515
  while(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK){ // check if conenction could be establisehd; retry if not
    Serial.println("Error Initializing MCP2515...");
    delay(100);
  } 
  Serial.println("MCP2515 Initialized Successfully!");
  CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted

  servo_conn.set_origin(ID_MOTOR, 0); // set zero position of motor until power-off
}

void loop() {
  servo_conn.set_duty_cycle(ID_MOTOR, 0.1);
  delay(5);
  // check for incoming messages
  if (!digitalRead(CAN0_INT)) {
    // get all messages in waiting queue
    while (CAN_MSGAVAIL == CAN0.checkReceive()) {
      // receive and process messages
      servo_conn.can_receive();
    }
  }
  // print received data to Serial output
  servo_conn.print_motor_vars(ID_MOTOR);
}
