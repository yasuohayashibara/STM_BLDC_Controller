#ifndef ADCONV_H
#define ADCONV_H

#include "main.h"
#include "stm32f7xx_hal.h"

class ADConv
{
public:
  ADConv(ADC_HandleTypeDef *hadc) :
    _hadc(hadc)
  {}
  
    
private:
  ADC_HandleTypeDef *_hadc;
};

#endif
