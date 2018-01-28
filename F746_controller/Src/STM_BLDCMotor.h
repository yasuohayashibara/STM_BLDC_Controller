// Chiba Institute of Technology

#ifndef STM_BLDCMOTOR_H
#define STM_BLDCMOTOR_H

#include "PWM.h"
#include "AS5600.h"
#include "Output.h"

/** Class to control a motor on any pin, without using pwm pin
 *
 * Example:
 * @code
 * // STM_BLDCMotor Control
 * #include "mbed.h"
 * #include "STM_BLDCMotor.h"
 *
 * STM_BLDCMotor motor;
 *
 * int main(){
 *   motor.servoOn();
 *   motor = 0.1;    // duty ratio
 * }
 * @endcode
 */


class STM_BLDCMotor
{
public:
    /** Create a new SoftwarePWM object on any mbed pin
      *
      * @param Pin Pin on mbed to connect PWM device to
     */
    STM_BLDCMotor(TIM_HandleTypeDef *htim, AS5600 *as5600);

    void servoOn(void);

    void servoOff(void);

    void setMaxDutyRatio(float max_ratio);

    void setPwmPeriod(double seconds);

    void write(double value);

    float read();
    
    int getHoleState();

    int getState();

    void status_changed(void);

    void resetHoleSensorCount();

//#ifdef MBED_OPERATORS
    /** A operator shorthand for write()
     */
    STM_BLDCMotor& operator= (float value) {
        write(value);
        return *this;
    }

    STM_BLDCMotor& operator= (STM_BLDCMotor& rhs) {
        write(rhs.read());
        return *this;
    }

    /** An operator shorthand for read()
     */
    operator float() {
        return read();
    }
//#endif

private:
    TIM_HandleTypeDef *_htim;
    PWM _uh;
    Output _ul;
    PWM _vh;
    Output _vl;
    PWM _wh;
    Output _wl;

    double _value;
    double _max_ratio;
    bool _enable;
    int _hole_state_no;
    float _hole_state0_angle;
    float _angle;
    float _integral_angle;
    float _prev_angle;

    AS5600 *_as5600;

    static int switching_table[6][3];
    void drive(int u, int v, int w);

    enum h_bridge{
      UH = 0,
      UL,
      VH,
      VL,
      WH,
      WL,
    };
};

#endif