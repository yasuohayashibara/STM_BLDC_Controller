#ifndef PWM_H
#define PWM_H

#include "main.h"
#include "stm32f7xx_hal.h"

class PWM
{
public:
  PWM(int no, TIM_HandleTypeDef *htim) :
    _no(no), _duty_ratio(0), _htim(htim)
  {
    HAL_TIM_PWM_Start(_htim, _channel[_no]);
  }
  
  void write(float duty) {
    _duty_ratio = (int)(duty * 100); 
    __HAL_TIM_SetCompare(_htim, _channel[_no], _duty_ratio);
  }
  
  float read() { return (float)_duty_ratio / 100.0f; }
  
  PWM& operator= (float value) {
    write(value);
    return *this;
  }
  
  operator float() { return read(); }
  
private:
  int _no;
  int _duty_ratio;
  TIM_HandleTypeDef *_htim;
  static const uint32_t _channel[3];
};

const uint32_t PWM::_channel[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};

#endif
