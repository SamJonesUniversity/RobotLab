//#include "TimerOne.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

//Set up the serial input/output
SoftwareSerial bluetooth(8,9);
//Set the magentometer ready for use
MPU6050 mpu;

//Pins for connected sensors
const int trigPin = 12;
const int echoPin = 13;
const int encoderInL = 2;
const int encoderInR = 3;
const int leftF = 4;
const int leftB = 5;
const int rightF = 6;
const int rightB = 7;

//Variables required for calculations
unsigned long timer = 0;
float increasetime = 0.01;
unsigned long currenttime = 0;
float yaw = 0;
long duration;
int distance;
float counterL = 0;
float counterR = 0;
int encoderLstate = 0;
int encoderRstate = 0;
char c = ' ';

//Function for working out rotations per second
int counted = 0;
int countedR = 0;
void HolesPerSecL()
{
  counterL++;
  counted = 1;
}
void HolesPerSecR()
{
  counterR++;
  countedR = 1;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Begin the serial monitor at 9600
  bluetooth.begin(9600); //Begin the bluetooth serial at 9600
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(leftF, OUTPUT); //Set pins for motors as an output
  pinMode(leftB, OUTPUT);
  pinMode(rightF, OUTPUT);
  pinMode(rightB, OUTPUT);
  pinMode(encoderInL, INPUT); //Set pins for wheel encoders
  pinMode(encoderInR, INPUT); //as input

  //--FOR SETTING UP MPU ONLY
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valud MPU6050 sensor");
    delay(500);
  }
  //Get the initial direction the robot is facing
  mpu.calibrateGyro();
}


void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
  // Prints the distance on the Serial Monitor
  //Serial.print("Distance: ");
  //Serial.println(distance);

  //Used to calculate the number of rotations of the wheel
  encoderLstate = digitalRead(encoderInL);
  encoderRstate = digitalRead(encoderInR);
  if(encoderLstate == HIGH && counted == 0)
  {
    HolesPerSecL();
  }
  if(encoderLstate == LOW)
  {
    counted = 0;
  }
  if(encoderRstate == HIGH && countedR == 0)
  {
    HolesPerSecR();
  }
  if(encoderRstate == LOW)
  {
    countedR = 0;
  }
  
  if(distance <= 50)
  {
    right();
  }
  else
  {
    forward();
  }

  //Get a time to return a value every second
  currenttime = millis();
  //unsigned long thistime = currenttime - timer;

  //Read normalised values
  Vector normalisedvalues = mpu.readNormalizeGyro();
  //Calculate yaw
  yaw = yaw + normalisedvalues.XAxis * increasetime;
  
  if((currenttime - timer) >= 1000)
  {
    
    //Serial.println();
    //Serial.print("RotationsL per second: ");
    //float rotationsLps = counterL/20;
    //float rotationsRps = counterR/20;
    //Serial.println(rotationsLps);
    //Serial.print("RotationsR per second: ");
    //Serial.println(rotationsRps);
    timer = currenttime;
    //counterL = 0;
    //counterR = 0;
    byte teststring = distance;
    //bluetooth.print("Distance fro2m front sensor: ");
    bluetooth.println(teststring);
   // bluetooth.print("\n");
    //bluetooth.print("Current rotation: ");
    bluetooth.println(yaw);
    //bluetooth.print("\n");
    
  }
  
    Serial.print("Current yaw: ");
    Serial.println(round(yaw));
    
    
    //delay((increasetime*1000) - (millis() - currenttime));
    
}

void forward()
{
  digitalWrite(leftF, HIGH);
  digitalWrite(leftB, LOW);
  digitalWrite(rightF, HIGH);
  digitalWrite(rightB, LOW);
}

void left()
{
  digitalWrite(leftF, HIGH);
  digitalWrite(leftB, LOW);
  digitalWrite(rightF, LOW);
  digitalWrite(rightB, HIGH);
}

void right()
{
  digitalWrite(leftF, LOW);
  digitalWrite(leftB, HIGH);
  digitalWrite(rightF, HIGH);
  digitalWrite(rightB, LOW);
}

void backward()
{
  digitalWrite(leftF, LOW);
  digitalWrite(leftB, HIGH);
  digitalWrite(rightF, LOW);
  digitalWrite(rightB, HIGH);
}

