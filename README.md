# Actuators and Sensors Library

This library allows you to control various acutators and sensors via an object level in arduino. All distance measurements are in CM

## Ultrasonic Sensor

A sonar based sensor that can return a distance. The constructor requires the trig and echo pins

```c++
#include <ActuatorsAndSensors.h>

#define TRIG 2
#define ECHO 3

//Instantiate the Sensor
UltrasonicSensor DistanceSensor(TRIG, ECHO);

void setup() {
  Serial.begin(9600);
}

void loop() {

  //returns the distance measured by polling the ultrasonic sensor
  int distance = DistanceSensor.read();
  
  Serial.println(distance);
}
```

## IR Sensor

A light based sensor that can return a distance. The constructor requires only the analog read pin

```c++
#include <ActuatorsAndSensors.h>

#define IR_PIN 2

//Instantiate the Sensor
IRSensor IR(IR_PIN);

void setup() {
  Serial.begin(9600);
}

void loop() {

  //returns the distance measured by the IR sensor in CM
  int distance = IR.read();
  
  Serial.println(distance);
}
```

## Bump Switch

```c++
#include <ActuatorsAndSensors.h>

#define BUMP_PIN 2

//Instantiate the Sensor as Normally Closed
BumpSwitch BS_NC(BUMP_PIN,"NC");

//Instantiate the Sensor as Normally Open
BumpSwitch BS_NO(BUMP_PIN,"NO");

void setup() {
  Serial.begin(9600);
}

void loop() {

  //returns if the bump switch is hit (1) or not (0)
  int isHit = BS.read();
  
  Serial.println(isHit);
}
```

## Linear Actuator

```c++
#include <ActuatorsAndSensors.h>

#define IN1 2 
#define IN2 3
#define ENA 4

//Instantiate the Linear Actuator
LinearActuator LinAct(IN1, IN2, ENA);

void setup() {
  
  //Allows the actuator to recieve power
  LinAct.turnOn();
}

void loop() {

  int pwm = 255;

  //Sends the actuator up at specified pwm (default value of 255 if not passed)
  LinAct.goUp(pwm);
  delay(5000);

  //Stops linear actuator motion until a goUp() or goDown() but doesn't turn it off
  LinAct.pause();
  delay(5000);

  //Sends the actuator down at specified pwm (default value of 255 if not passed)
  LinAct.goDown(pwm);
  delay(5000);

  //Turns the linear actuator off, goUp() and goDown() will not work until turnOn() is called
  LinAct.turnOff();
}
```

## Motor

Basic motor example. If you want any feedback or functions not listed below, internalUpdate() MUST be called in loop without any significant delay functions

```c++
#include <ActuatorsAndSensors.h>

#define IN1 2 
#define IN2 3
#define ENA 4

#define ENCA 8
#define ENCB 9

#define BUMP 10

//Instantiate the Motor's Encoder via Encoder.h
Encoder enc(ENCA, ENCB);

//Motor input bounds
int pwmLower=0;
int pwmUpper=255;
long countPerCm=100;

//Instantiate the Motor
Motor motor(IN1, IN2, ENA, pwmLower, pwmUpper, &enc, countPerCm);

void setup() {
  Serial.begin(9600);
  
  //Allows the motor to recieve power
  motor.turnOn();
}

void loop() {

  int pwm = 255;

  //Sends the motor forwards at specified pwm
  motor.goForward(pwm);
  delay(5000);

  //Stops motor motion until a goForward(), goBackward(), or other subsequent acutation functions are called, but doesn't turn it off
  motor.pause();
  delay(5000);

  //Sends the motor backwards at specified pwm
  motor.goBackward(pwm);
  delay(5000);

  //Turns the motor off, actuation functions will not work until turnOn() is called
  motor.turnOff();
}
```

Feedback example with internalUpdate()

```c++
#include <ActuatorsAndSensors.h>

#define IN1 3 
#define IN2 4
#define ENA 2

#define ENCA 18
#define ENCB 19

#define BUMP 53

//Instantiate the Motor's Encoder via Encoder.h
Encoder enc(ENCA, ENCB);

//Motor input bounds
int pwmLower=0;
int pwmUpper=255;
long countPerCm=10;

//Instantiate the Motor
Motor motor(IN1, IN2, ENA, pwmLower, pwmUpper, &enc, countPerCm);

//Bump Switch object for "until" functions
BumpSwitch BS(BUMP,"NC");

//mode variable to try different execution options. Demo purpose only
int mode = 1;

//demo control gains
double kp = 0.2;
double ki = 0.00005;
double kd = 200;
double posTarget = 1000;

double kp_vel = 20;
double ki_vel = 0.1;
double kd_vel = 0.001;
double velTarget = 0.2;

double initPosition = 100;

void setup() {
  Serial.begin(9600);
  
  //Allows the motor to recieve power
  motor.turnOn();

  int pwm = 255;


  switch(mode){
    case 1:
      //Set the motor to move forward until the bump switch is hit
      motor.goForwardUntil(pwm, &BS, "==", 1);
      break;
    case 2:
      //Set the motor to move backward until the bump switch is hit
      motor.goBackwardUntil(pwm, &BS, "==", 1);
      break;
    case 3:
      //Set the current position to a specified value - useful to reset before PID position tracking
      motor.setPos(initPosition);
    
      //Set the motor to track the position target with given PID gains
      motor.PIDPos(kp, ki, kd, posTarget);
      break;
    case 4:
      //Set the motor to track the velocity target with given PID gains
      motor.PIDVel(kp_vel, ki_vel, kd_vel, velTarget);
      break;
    case 5:
      //Set the motor to hold at current position
      motor.setPos(0);
      motor.PIDPos(2, 0, 0, 0);
      break;
  }

}


void loop() {
  //perform the motor update required for PID, until movemenent, and any feedback
  motor.internalUpdate();

  //print out position
  Serial.print("Pos: ");
  Serial.print(motor.getPos());

  //print out velocity
  Serial.print(" Vel: ");
  Serial.print(motor.getVel(),4);

  //print out acceleration
  Serial.print(" Acc: ");
  Serial.println(motor.getAcc(),6);
  
}
```

## Function List

```c++
// Ultrasonic sensor that utilizes an output and input pin
UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin);

//Read the Ultrasonic Sensor
int UltrasonicSensor::read();

// IR sensor that uses only an analog pin
IrSensor::IrSensor(int readPin);

//Read the IR sensor
int IrSensor::read();

//Digital pin and a string of "NC" or "NO" to denote normally open or normally closed operation.
BumpSwitch::BumpSwitch(int pin, String type);

//Returns 1 if pressed, regardless of NO or NC
int BumpSwitch::read();

//Linear actuator constructor with two input pins and a pwm pin
LinearActuator::LinearActuator(int IN1, int IN2, int ENA);

//Makes the actuator extend at pwm speed (default is at full speed via the class definition)
void LinearActuator::goUp(int pwm);

//Makes the actuator lower at pwm speed (default is at full speed via the class definition)
void LinearActuator::goDown(int pwm);

//Pause (but not turn off) the linear actuator
void LinearActuator::pause();

//Enable all motion methods. Required to run at start, or after turning off
void LinearActuator::turnOn();

//Prevent the linear actator from being actuated in any direction until this->turnOn() is called
void LinearActuator::turnOff();

//Class Instantiator. Pass the three pins, upper and lower pwm limits, a pointer of an encoder object from Encoder.h (e.g., Encoder* myEnc(18,19)), and the number of encoder counts per cm of tangential motion (i.e., counts per rev*rev/cm)
Motor::Motor(int IN1, int IN2, int ENA, int lowerLim, int upperLim, Encoder* enc, double countPerCm);

//MUST call in main loop - Performs all internal function updates
void Motor::internalUpdate();

//Setup PID Position Tracking to the given target
void Motor::PIDPos(double kp, double ki, double kd, double target);

//Setup PID Velocity Tracking to the given target
void Motor::PIDVel(double kp, double ki, double kd, double target);

//Return motor position
double Motor::getPos();

//Return motor velocity
double Motor::getVel();

//return motor acceleration
double Motor::getAcc();

//Set motor to be able to recieve current
void Motor::turnOn();

//Prevent motor from recieving current
void Motor::turnOff();

//change the internal position mode to the specified parameter
void Motor::setPos(double position);

//Go forward at pwm/255% power - open loop forced
void Motor::goForward(int pwm);

//Go backward at pwm/255% power - open loop forced
void Motor::goBackward(int pwm);

//Stop motion but keep motor in an active state - open loop forced
void Motor::pause();

//Go forward until event happens as on specified sensor pointer, under contition (i.e., "==","!=","<",...) compared to value
void Motor::goForwardUntil(int pwm, Sensor *trigger, String condition ,int value);

//Go backward until event happens as on specified sensor pointer, under contition (i.e., "==","!=","<",...) compared to value
void Motor::goBackwardUntil(int pwm, Sensor *trigger, String condition ,int value);
```
