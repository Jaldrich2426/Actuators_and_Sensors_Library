/**************************************************************************************
****************************************************************************************
******************************Actuators and Sensors Library*****************************
****************************************************************************************
***************************************************************************************/

//This library has a combination of actuators and sensors
//All units are in cm, so if you want different units, use a common conversion factor
#ifndef ActuatorsAndSensors_h
#define ActuatorsAndSensors_h

#include "Arduino.h"
#include <Encoder.h>

//Base class for sensor types to utilize a common read function for trigger events
class Sensor{
  public:
    virtual int read() = 0;
};

/**************************************************************************************
********************************Ultrasonic Sensor Class********************************
***************************************************************************************/

class UltrasonicSensor : public Sensor{
  public:
    UltrasonicSensor(int trigPin, int echoPin);
    int read();
  private:
    int _trigPin;
    int _echoPin;
};


/**************************************************************************************
************************************IR Sensor Class************************************
***************************************************************************************/

//Sharp IR sensor class. Returns value according to decaying exponential betwen 10-80cm
class IrSensor : public Sensor{
  public:
    IrSensor(int readPin);
    int read();
  private:
    int _readPin;
};

/**************************************************************************************
***********************************Bump Switch Class***********************************
***************************************************************************************/

//Simple bump switch class. Would also work with regular switches.
class BumpSwitch : public Sensor{
  public:
    BumpSwitch(int pin, String type);
    int read();
  private:
    int _pin;
    String _type;
    bool _NC;
};


/**************************************************************************************
*********************************Linear Actuator Class*********************************
***************************************************************************************/

//Simple Linear actuator class, no direct feedback
class LinearActuator {
  public:
    LinearActuator(int IN1, int IN2, int ENA);
    void goUp(int pwm = 255);
    void goDown(int pwm = 255);
    void pause();
    void turnOn();
    void turnOff();
  private:
    int _IN1;
    int _IN2;
    int _ENA;
    bool _active;
};

/**************************************************************************************
**************************************Motor Class**************************************
***************************************************************************************/

//DC motor class with 2 channel encoder attached
class Motor{
  public:
    Motor(int IN1, int IN2, int ENA, int lowerLim, int upperLim, Encoder* enc, double countPerCm);

    //Open Loop Methods
    void goForward(int pwm);
    void goBackward(int pwm);
    void goForwardUntil(int pwm, Sensor *trigger, String condition ,int value);
    void goBackwardUntil(int pwm, Sensor *trigger, String condition ,int value);
    void pause();

    //Power Control
    void turnOn();
    void turnOff();

    //Closed Loop Methods
    void PIDPos(double kp, double ki, double kd, double target);
    void PIDVel(double kp, double ki, double kd, double target);
    
    //Direct Read Feedback
    double getPos();
    double getVel();
    double getAcc();

    //Reseting Position
    void setPos(double position); //done

    //Main Loop Function that MUST be called in loop() without significant delay functions (runs on a 50 microsecond clock)
    void internalUpdate(); //needs finishing

  private:
    //Pins
    int _IN1;
    int _IN2;
    int _ENA;

    //PWM Bounds
    int _lowerLim;
    int _upperLim;

    //Motor Geometry
    double _countPerCm;

    //State Variables
    double _position;
    double _velocity;
    double _acceleration;

    //If power is allowed to motor or not
    bool _active;

    //PID Variables
    bool _posTrack; //tracking velocity if false
    double _posTarget;
    double _velTarget;
    double _kp;
    double _kd;
    double _ki;
    bool _closedLoop;
    double _iError;
    double _startError;

    //Encoder object pointer
    Encoder* _encoder = NULL;  

    //InternalUpdate() variables for calculations
    int _polingInterval;
    unsigned long _startTime;
    unsigned long _currentTime;
    double _startPosition;
    double _startVelocity;

    //Event Trigger Variables
    bool _checkTrigger;
    Sensor* _trigger;
    int _triggerValue;
    String _condition;     

    //Internal Functions
    void goForwardInternal(int pwm); //done
    void goBackwardInternal(int pwm); //done
    void pauseInternal(); //done
};

#endif