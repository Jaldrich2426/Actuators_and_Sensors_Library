/**************************************************************************************
****************************************************************************************
******************************Actuators and Sensors Library*****************************
****************************************************************************************
***************************************************************************************/

//This library has a combination of actuators and sensors
//All units are in cm, so if you want different units, use a common conversion factor

#include "Arduino.h"
#include "ActuatorsAndSensors.h"
#include <Encoder.h>
#define ENCODER_OPTIMIZE_INTERRUPTS

/**************************************************************************************
********************************Ultrasonic Sensor Class********************************
***************************************************************************************/

// Ultrasonic sensor that utilizes an output and input pin
UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin){
  _trigPin=trigPin;
  _echoPin=echoPin;
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
}

int UltrasonicSensor::read(){
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(_echoPin, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  return distance;
}

/**************************************************************************************
************************************IR Sensor Class************************************
***************************************************************************************/

IrSensor::IrSensor(int readPin){
  _readPin=readPin;
};

int IrSensor::read(){
  float volts = analogRead(_readPin)*0.0048828125;  // value from sensor * (5/1024)
  int distance = distance = 29.988*pow(volts , -1.173); // worked out from datasheet graph

  //cap return at 10-80cm
  return (distance <= 80 && distance >=10) ? distance : -1;
}

/**************************************************************************************
***********************************Bump Switch Class***********************************
***************************************************************************************/

//Digital pin and a string of "NC" or "NO" to denote normally open or normally closed operation.
BumpSwitch::BumpSwitch(int pin, String type){
  _pin=pin;
  pinMode(_pin,INPUT_PULLUP);
  _type=type;
  _NC = type.equals("NC");
}

//Returns 1 if pressed, regardless of NO or NC
int BumpSwitch::read(){
  return (_NC) ? digitalRead(_pin) : !digitalRead(_pin);
}

/**************************************************************************************
*********************************Linear Actuator Class*********************************
***************************************************************************************/

//Constructor with two input pins and a pwm pin
LinearActuator::LinearActuator(int IN1, int IN2, int ENA){
  _IN1=IN1;
  _IN2=IN2;
  _ENA=ENA;
  _active=false;
}

//Makes the actuator extend at pwm speed (default is at full speed via the class definition)
void LinearActuator::goUp(int pwm){
  if(_active){
    digitalWrite(_IN1,HIGH);
    digitalWrite(_IN2,LOW);
    analogWrite(_ENA,pwm);
  }
}

//Makes the actuator lower at pwm speed (default is at full speed via the class definition)
void LinearActuator::goDown(int pwm){
  if(_active){
    digitalWrite(_IN1,LOW);
    digitalWrite(_IN2,HIGH);
    analogWrite(_ENA,pwm);
  }
}

//Pause (but not turn off) the linear actuator
void LinearActuator::pause(){
  digitalWrite(_IN1,LOW);
  digitalWrite(_IN2,LOW);
  analogWrite(_ENA,0);
}

//Enable all motion methods. Required to run at start, or after turning off
void LinearActuator::turnOn(){
  _active=true;
}

//Prevent the linear actator from being actuated in any direction until this->turnOn() is called
void LinearActuator::turnOff(){
  pause();
  _active=false;
}

String LinearActuator::getLog(){
  String log="";

  log += _active;
  log += ",";
  log += _effort;
  return log;
}
/**************************************************************************************
**************************************Motor Class**************************************
***************************************************************************************/

//Class Instantiator. Pass the three pins, upper and lower pwm limits, a pointer of an encoder object from Encoder.h (e.g., Encoder* myEnc(18,19)), and the number of encoder counts per cm of tangential motion (i.e., counts per rev*rev/cm)
Motor::Motor(int IN1, int IN2, int ENA, int lowerLim, int upperLim, Encoder* enc, double countPerCm,int encDir){
  //Define Pins
  _IN1=IN1;
  _IN2=IN2;
  _ENA=ENA;

  //PWM Limits
  _lowerLim=lowerLim;
  _upperLim=upperLim;

  //Set object pointer to encoder
  _encoder = enc;
  _encDir = encDir;

  //Motor Geometry
  _countPerCm=countPerCm;

  //Initialize position, velocity, and acceleration variables
  _position=0;
  _velocity=0;
  _acceleration=0;
  _startPosition=0;
  _startVelocity=0;

  //Set motor to inactive to start
  _active = false;

  //Set update variables
  _posTrack = true;
  _polingInterval = 50;

  //Initialzie Timings
  _startTime=millis();
  _currentTime=_startTime;

  //Initialize No Triggers
  _checkTrigger=false;


}

//MUST call in main loop - Performs all internal function updates
void Motor::internalUpdate(){
  //update position
  _position=getPos();

  _currentTime=millis();
  //50 microsecond timer (or _polintInverval Microseconds)
  if ( _currentTime - _startTime >= _polingInterval ) {
    // time to calculate average encoder speed
    _velocity = (_position - _startPosition) / (double)(_currentTime - _startTime);
    _startPosition = _position;

    // calculate acceleration
    _acceleration = (_velocity - _startVelocity) / (double)(_currentTime - _startTime);
    _startVelocity = _velocity;
    
    //closed loop calculations
    if(_closedLoop){
      int output;   
      if(_posTrack){                                             //Tracking Position

        //Error calculations
        double pError = _posTarget-_position;
        _iError += (double)(pError*(double)(_currentTime - _startTime));
        double dError = (pError-_startError)/(double)(_currentTime - _startTime);
        _startError = pError;

        //Calculate control effort
        output = (int)(_kp*pError +_ki*_iError + _kd*dError);
      }else{                                                     //Tracking Velocity

        //Error Calculations
        double pError = _velTarget-_velocity;
        _iError += (double)(pError*(double)(_currentTime - _startTime)); 
        double dError = (pError-_startError)/(double)(_currentTime - _startTime);
        _startError=pError;

        //Calculate control effort
        output = (int)(_kp*pError +_ki*_iError + _kd*dError);
      }

      //Set motor to appropriate mode/speed
      if(output > 0){
        goForwardInternal(output);
      }else if(output < 0){
        goBackwardInternal(-output);
      }else{
        pauseInternal();
      }
    }
    //reset time
    _startTime = _currentTime;
  }

  //Tf we are checking triggers, read the value and check according to the condition. If met, pause the motor and stop checking triggers
  if(_checkTrigger){
    int value = _trigger->read();
    if(_condition.equals("==")){
      if(value == _triggerValue){
        _checkTrigger=false;
      }
    }else if(_condition.equals("!=")){
      if(value != _triggerValue){
        _checkTrigger=false;
      }
    }else if(_condition.equals(">=")){
      if(value >= _triggerValue){
        _checkTrigger=false;
      }
    }else if(_condition.equals(">")){
      if(value > _triggerValue){
        _checkTrigger=false;
      }
    }else if(_condition.equals("<=")){
      if(value <= _triggerValue){
        _checkTrigger=false;
      }
    }else if(_condition.equals("<")){
      if(value < _triggerValue){
        _checkTrigger=false;
      }
    }
    if(!_checkTrigger){
      pause();
    }
  }
}

//Setup PID Position Tracking to the given target
void Motor::PIDPos(double kp, double ki, double kd, double target){
  //Gains
  _kp=kp;
  _ki=ki;
  _kd=kd;
  //Mode
  _posTrack=true;
  //Setpoint
  _posTarget=target;
  //Initialize PID mode variables
  _closedLoop=true;
  _iError = 0;
  _startError = 0;
}

//Setup PID Velocity Tracking to the given target
void Motor::PIDVel(double kp, double ki, double kd, double target){
  //Gains
  _kp=kp;
  _ki=ki;
  _kd=kd;
  //Mode
  _posTrack=false;
  //Setpoint
  _closedLoop=true;
  //Initialize PID mode variables
  _velTarget=target;
  _iError = 0;
  _startError = 0;
}

//Return motor position
double Motor::getPos(){
  _position= _encDir*_encoder->read()/_countPerCm;
  return _position;
}

//Return motor velocity
double Motor::getVel(){
  return _velocity;
}

//return motor acceleration
double Motor::getAcc(){
  return _acceleration;
}

//Set motor to be able to recieve current
void Motor::turnOn(){
  _active=true;
}

//Prevent motor from recieving current
void Motor::turnOff(){
  pause();
  _active=false;
  delay(1);
}

//change the internal position mode to the specified parameter
void Motor::setPos(double position){
  _encoder->write(position*_countPerCm);
  _position=position;
}

//Go forward at pwm/255% power - open loop forced
void Motor::goForward(int pwm){
  _closedLoop=false;
  goForwardInternal(pwm);
}

//Go backward at pwm/255% power - open loop forced
void Motor::goBackward(int pwm){
  _closedLoop=false;
  goBackwardInternal(pwm);
}

//Go forward at pwm/255% power - Internal call
void Motor::goForwardInternal(int pwm){
  if(_active){
    pwm = (pwm > _upperLim) ? _upperLim : pwm;
    pwm = (pwm < _lowerLim) ? _lowerLim : pwm;
    digitalWrite(_IN1,LOW);
    digitalWrite(_IN2,HIGH);
    analogWrite(_ENA,pwm);
    _effort=pwm;
  }
}

//Go backward at pwm/255% power - internal call
void Motor::goBackwardInternal(int pwm){
  if(_active){
    pwm = (pwm > _upperLim) ? _upperLim : pwm;
    pwm = (pwm < _lowerLim) ? _lowerLim : pwm;
    digitalWrite(_IN1,HIGH);
    digitalWrite(_IN2,LOW);
    analogWrite(_ENA,pwm);
    _effort=-pwm;
  }
}

//Stop motion but keep motor in an active state - open loop forced
void Motor::pause(){
  _closedLoop=false;
  pauseInternal();
}

//Stop motion but keep motor in an active state - internal call
void Motor::pauseInternal(){ //done
  digitalWrite(_IN1,LOW);
  digitalWrite(_IN2,LOW);
  analogWrite(_ENA,0);
  _effort=0;
}

//Go forward until event happens as on specified sensor pointer, under contition (i.e., "==","!=","<",...) compared to value
void Motor::goForwardUntil(int pwm, Sensor *trigger, String condition ,int value){
  _closedLoop=false;
  goForwardInternal(pwm);

  //set trigger parameters
  _trigger=trigger;
  _condition=condition;
  _triggerValue=value;
  _checkTrigger=true;
}

//Go backward until event happens as on specified sensor pointer, under contition (i.e., "==","!=","<",...) compared to value
void Motor::goBackwardUntil(int pwm, Sensor *trigger, String condition ,int value){
  _closedLoop=false;
  goBackwardInternal(pwm);

  //set trigger parameters
  _trigger=trigger;
  _condition=condition;
  _triggerValue=value;
  _checkTrigger=true;
}

String Motor::getLog(){
  String log="";

  log += _active;
  log += ",";
  log += _position;
  log += ",";
  log += _velocity;
  log += ",";
  log += _acceleration;
  log += ",";
  log += _effort;
  log += ",";
  log += _posTrack;
  log += ",";
  if(_posTrack){
    log += _posTarget;
  }else{
    log += _velTarget;
  }
  log += ",";
  return log;
}