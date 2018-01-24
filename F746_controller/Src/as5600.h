#ifndef AS5600_H
#define AS5600_H

#include "main.h"
#include "stm32f7xx_hal.h"

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

class AS5600
{
public:
  AS5600(I2C_HandleTypeDef *hi2c);
    
  void setOffserAngleRad(float value) { _angle0 = value; }
  
  void startMeasure();
  
  void stopMeasure();
  
  float read() { return _angle; }
  
  operator float() { return read(); }
  
  float getAngleRad() { return _angle; }
  
  float getAngleDeg() { return _angle * 180.0f / M_PI; }
  
  bool getError() { return _error; }
  
  void resetError() { _error = false; }
  
  bool sendMeasureAngleRequest();
  
  bool receiveAngleRequest();
  
  void receiveAngle();

  bool _do_measure;
  
  bool _error;
  
private:
  float _angle;
  float _angle0;

  float maxPI(float angle);
  
  I2C_HandleTypeDef *_hi2c;

  static const int TX_BUF_SIZE = 16;
  static const int RX_BUF_SIZE = 16;
  unsigned char _tx_buf[TX_BUF_SIZE];
  unsigned char _rx_buf[RX_BUF_SIZE];

  static const int SLAVE_ADRESS = 0x36;
  static const int ANGLE_ADRESS = 0x0E;
};

#endif
