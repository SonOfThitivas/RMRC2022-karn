#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
int x, degreeBottom = 0, degreeTop = 0, degreeMouth = 0;


//left motor wheels
#define IN1leftfront 6 //esp32 = 27 14 12 13 16 4 2 15 esp8266 = 16 5 4 0 14 12 13 15
#define IN2leftfront 7
#define IN3leftback 8
#define IN4leftback 9

//right motor wheels
#define IN1rightfront 2
#define IN2rightfront 3
#define IN3rightback 4
#define IN4rightback 5

//Servo+
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 125
#define SERVOMAX 575  //Try to find max and min angle servo
uint8_t servoBottom = 13, servoTop = 14, servoMouth = 15;


void setup() {

  pinMode(IN1leftfront, OUTPUT);
  pinMode(IN2leftfront, OUTPUT);
  pinMode(IN3leftback, OUTPUT);
  pinMode(IN4leftback, OUTPUT);
  pinMode(IN1rightfront, OUTPUT);
  pinMode(IN2rightfront, OUTPUT);
  pinMode(IN3rightback, OUTPUT);
  pinMode(IN4rightback, OUTPUT);
  Stop();
  //Servo
  pwm.begin();
  pwm.setPWMFreq(60);
  //yield();

  delay(3000);
  Serial.begin(9600);
  Serial.setTimeout(1);
}
void loop() {
  while (!Serial.available());
  x = Serial.readString().toInt();
  CarMovement();
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //ServoBottom press 1
  if (x == 5) {
    while (true) {
      while (!Serial.available());
      x = Serial.readString().toInt();
      if (x == 10) { // press q to check servo
        break;
      }
      CarMovement();

      //Servo control
      if (x == 8) {
        if (degreeBottom < 180) {
          degreeBottom++;
        }
        pwm.setPWM(servoBottom, 0, AngleToPulse(degreeBottom));
      } else if (x == 9) {
        if (degreeBottom > 0) {
          degreeBottom--;
        }
        pwm.setPWM(servoBottom, 0, AngleToPulse(degreeBottom));
      }

    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // ServoTop press 2
  else if (x == 6) {
    while (true) {
      while (!Serial.available());
      x = Serial.readString().toInt();
      if (x == 10) { // press q to check servo
        break;
      }
      CarMovement();

      //Servo control
      if (x == 8) {
        if (degreeTop < 180) {
          degreeTop++;
        }
        pwm.setPWM(servoTop, 0, AngleToPulse(degreeTop));
      } else if (x == 9) {
        if (degreeTop > 0) {
          degreeTop--;
        }
        pwm.setPWM(servoTop, 0, AngleToPulse(degreeTop));
      }

    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // ServoMouth press 3
  else if (x == 7) {
    while (true) {
      while (!Serial.available());
      x = Serial.readString().toInt();
      if (x == 10) { // press q to check servo
        break;
      }
      CarMovement();

      //Servo control
      if (x == 8) {
        if (degreeMouth < 180) {
          degreeMouth++;
        }
        pwm.setPWM(servoMouth, 0, AngleToPulse(degreeMouth));
      } else if (x == 9) {
        if (degreeMouth > 0) {
          degreeMouth--;
        }
        pwm.setPWM(servoMouth, 0, AngleToPulse(degreeMouth));
      }

    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////
}

void forward()  //1
{
  //leftfrontwheel
  digitalWrite(IN1leftfront, HIGH);
  digitalWrite(IN2leftfront, LOW);
  //leftbackwheel
  digitalWrite(IN3leftback, HIGH);
  digitalWrite(IN4leftback, LOW);
  //rightfrontwheel
  digitalWrite(IN1rightfront, HIGH);
  digitalWrite(IN2rightfront, LOW);
  //rightbackwheel
  digitalWrite(IN3rightback, HIGH);
  digitalWrite(IN4rightback, LOW);
}

void backward() //2
{
  //leftfrontwheel
  digitalWrite(IN1leftfront, LOW);
  digitalWrite(IN2leftfront, HIGH);
  //leftbackwheel
  digitalWrite(IN3leftback, LOW);
  digitalWrite(IN4leftback, HIGH);
  //rightfrontwheel
  digitalWrite(IN1rightfront, LOW);
  digitalWrite(IN2rightfront, HIGH);
  //rightbackwheel
  digitalWrite(IN3rightback, LOW);
  digitalWrite(IN4rightback, HIGH);
}

void right() //4
{
  //leftfrontwheel
  digitalWrite(IN1leftfront, HIGH);
  digitalWrite(IN2leftfront, LOW);
  //leftbackwheel
  digitalWrite(IN3leftback, HIGH);
  digitalWrite(IN4leftback, LOW);
  //rightfrontwheel
  digitalWrite(IN1rightfront, LOW);
  digitalWrite(IN2rightfront, HIGH);
  //rightbackwheel
  digitalWrite(IN3rightback, LOW);
  digitalWrite(IN4rightback, HIGH);
}

void left() //3
{
  //leftfrontwheel
  digitalWrite(IN1leftfront, LOW);
  digitalWrite(IN2leftfront, HIGH);
  //leftbackwheel
  digitalWrite(IN3leftback, LOW);
  digitalWrite(IN4leftback, HIGH);
  //rightfrontwheel
  digitalWrite(IN1rightfront, HIGH);
  digitalWrite(IN2rightfront, LOW);
  //rightbackwheel
  digitalWrite(IN3rightback, HIGH);
  digitalWrite(IN4rightback, LOW);
}

void Stop() //0
{
  //leftfrontwheel
  digitalWrite(IN1leftfront, LOW);
  digitalWrite(IN2leftfront, LOW);
  //leftbackwheel
  digitalWrite(IN3leftback, LOW);
  digitalWrite(IN4leftback, LOW);
  //rightfrontwheel
  digitalWrite(IN1rightfront, LOW);
  digitalWrite(IN2rightfront, LOW);
  //rightbackwheel
  digitalWrite(IN3rightback, LOW);
  digitalWrite(IN4rightback, LOW);
}

int AngleToPulse(int ang) { //angle to pulse
  int pulse = map(ang, 0, 180, SERVOMIN , SERVOMAX);
  Serial.print("Angle: "); Serial.print(ang);
  Serial.print(" pulse: "); Serial.println(pulse);
  return pulse;
}

void CarMovement() {
  if (x == 1) { //forward
    Serial.println(x);
    forward();
  } else if (x == 2) { //backward
    Serial.println(x);
    backward();
  } else if (x == 3) { //left
    Serial.println(x);
    left();
  } else if (x == 4) { //right
    Serial.println(x);
    right();
  } else if (x == 0) { //stop
    Serial.println(x);
    Stop();
  } else {  //stop
    Stop();
  }
}
