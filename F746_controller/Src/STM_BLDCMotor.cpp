#include <math.h>
#include "STM_BLDCMotor.h"

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define TEST_MOTOR_HOLE0_ANGLE 3.068711

#define HOLE_STATE0   0x05  // 101  ( 0deg - 60deg)
#define HOLE_STATE1   0x04  // 100  ( 60deg - 120deg)
#define HOLE_STATE2   0x06  // 110  ( 120deg - 180deg)
#define HOLE_STATE3   0x02  // 010  ( 180deg - 240deg)
#define HOLE_STATE4   0x03  // 011  ( 240deg - 300deg)
#define HOLE_STATE5   0x01  // 001  ( 300deg - 360deg)

#define MIN_PWM 0.10
#define PWM_FREQUENCY 20000.0
#define SAMPLING_TIME 0.0001

float maxPI(float angle_rad)
{
  while(angle_rad >  M_PI) angle_rad -= 2 * M_PI;
  while(angle_rad < -M_PI) angle_rad += 2 * M_PI;
  return angle_rad;
}

int STM_BLDCMotor::switching_table[6] [3] = {
    { 0, -1, 1 }, // STATE1
    { 1, -1, 0 }, // STATE2
    { 1, 0, -1 }, // STATE3
    { 0, 1, -1 }, // STATE4
    { -1, 1, 0 }, // STATE5
    { -1, 0, 1 }, // STATE6
};

STM_BLDCMotor::STM_BLDCMotor(TIM_HandleTypeDef *htim, AS5600 *as5600) :
  _htim(htim),
  _uh(_htim, TIM_CHANNEL_1), _ul(L1_GPIO_Port, L1_Pin),
  _vh(_htim, TIM_CHANNEL_2), _vl(L2_GPIO_Port, L2_Pin),
  _wh(_htim, TIM_CHANNEL_3), _wl(L3_GPIO_Port, L3_Pin),
  _value(0.0f), _max_ratio(0.5f), _enable(false), 
  _hole_state_no(0), _hole_state0_angle(TEST_MOTOR_HOLE0_ANGLE),
  _angle(0), _integral_angle(0), _prev_angle(0),
  _as5600(as5600)
{
  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_3);

  _ul = _uh = _vl = _vh = _wl = _wh = 0;
  
  this->write(0);
}

void STM_BLDCMotor::servoOn(void)
{
  _enable = true;
}

void STM_BLDCMotor::servoOff(void)
{
  _enable = false;
}

void STM_BLDCMotor::setMaxDutyRatio(float max_ratio)
{
  _max_ratio = max(min(max_ratio, 1.0f), 0.0f);
  status_changed();
}

void STM_BLDCMotor::write(double value)
{
  _value = max(min(value, _max_ratio), -_max_ratio);
  status_changed();
}

float STM_BLDCMotor::read()
{
  return _value;
}

bool STM_BLDCMotor::update()
{
  const float angle_width = 2 * M_PI / 42;
  _angle = _hole_state0_angle + angle_width/2 + 2 * M_PI - _as5600->getAngleRad();
  float angle_diff = maxPI(_angle - _prev_angle);
  _prev_angle = _angle;
  _integral_angle += angle_diff;
  int hole_no = (int)(_angle / angle_width);
  _hole_state_no = hole_no % 6;

  return true;
}

int STM_BLDCMotor::getHoleState()
{
  return _hole_state_no;
}

float STM_BLDCMotor::getIntegratedAngleRad()
{
  return _integral_angle;
}

void STM_BLDCMotor::status_changed(void)
{
  int dir = (_value >= 0.0f) ? 1 : -2;
  int next_state = (_hole_state_no + dir + 6) % 6;
  if (_enable){
    drive(switching_table[next_state][0],
            switching_table[next_state][1],
            switching_table[next_state][2]);
  } else {
    drive(0, 0, 0);
  }
}

/*!
 * @brief drive for three phase motor
* @param[in] u switch u line (1:High, 0: NC, -1: Low)
* @param[in] v switch v line (1:High, 0: NC, -1: Low)
* @param[in] w switch w line (1:High, 0: NC, -1: Low)
 */
void STM_BLDCMotor::drive(int u, int v, int w)
{
  // prevent through current
  double val = fabs(_value);

  _uh = (u == 1) ? val : 0.0;
  _ul = (u == -1) ? 1 : 0;
  _vh = (v == 1) ? val : 0.0;
  _vl = (v == -1) ? 1 : 0;
  _wh = (w == 1) ? val : 0.0;
  _wl = (w == -1) ? 1 : 0;
}
