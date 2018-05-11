#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>


Servo left;
Servo right;

#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055();

void leftForward(Servo left){
  left.write(180);
}
void leftBack(Servo left)
{
  left.write(0);
}

void rightForward(Servo right)
{
  right.write(180);
}

void rightBack(Servo right)
{
  right.write(0);
}

void stop(Servo right, Servo left)
{
  right.write(90);
  left.write(90);
}
void setup() {
  Serial.begin(115200);
  left.attach(9);
  right.attach(10);
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("[");
  Serial.print(360-euler.x());
  Serial.println("]");
  if(Serial.available())
  {
    char c = Serial.read();
    if(c=='f')
    {
      digitalWrite(2, HIGH);
      digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      rightForward(right);
      leftForward(left);
      
    }
    if(c=='l')
    {
      digitalWrite(2, LOW);
      digitalWrite(3, HIGH);
      digitalWrite(4, LOW);
      rightForward(right);
      leftBack(left);
    }
    if(c=='r')
    {
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      digitalWrite(4, HIGH);
      rightBack(right);
      leftForward(left);
    }
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
