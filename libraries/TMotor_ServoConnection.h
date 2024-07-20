#ifndef TMOTOR_SERVOCONNECTION_H
#define TMOTOR_SERVOCONNECTION_H

#include "Arduino.h"
#include <mcp_can.h>
#include <map>

struct motor_data
{
  float position;
  float velocity;
  float current;
  int temp;
  int error;
};

struct motor_constants
{
  float Kt;
  float reduction_ratio;
};

class TMotor_ServoConnection
{
public:
  TMotor_ServoConnection(MCP_CAN &can_connection);

  void send_CAN_message(uint32_t id, bool extended, byte data[8], uint8_t len);
  void set_duty_cycle(long unsigned int id, float duty);
  void set_current(long unsigned int id, float current);
  void set_break(long unsigned int id, float current);
  void set_origin(long unsigned int id, uint8_t set_origin_mode);
  void set_pos(long unsigned int id, float pos);
  void set_speed(long unsigned int id, float speed);
  void set_pos_spd(long unsigned int id, float pos, int16_t spd, int16_t RPA);
  void can_receive(void);
  void parse_CAN_data(long unsigned int id, unsigned char rx_message[8]);
  motor_data *const get_motor_data_reference(int id);
  void print_motor_vars(int id);
  float get_pos(int id);
  float get_speed(int id);
  float get_current(int id);
  int get_temp(int id);
  int get_error(int id);
  float current_to_torque(float current);
  float torque_to_current(float torque);
  void wait_until_reached(long unsigned int id, float pos);

private:
  MCP_CAN &_can_connection;
  motor_constants motor_constants;
  std::map<int, motor_data> motors;
};

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
void buffer_append_int16(uint8_t *buffer, int16_t number, int16_t *index);

#endif
