#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <Stepper.h>

//Set up the serial input/output
SoftwareSerial bluetooth(8,9);

//Set the magentometer ready for use
MPU6050 mpu;

//Pins for connected sensors
const int trigPinF = 12;
const int echoPinF = 13;
const int trigPinL = 10;
const int echoPinL = 11;
const int trigPinR = A2;
const int echoPinR = A3;
const int trigPinB = A1;
const int echoPinB = A0;
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
float counterL = 0;
float counterR = 0;
int encoderLstate = 0;
int encoderRstate = 0;
const float circumference = 20.263;
bool stopped = false;
float rotationsLps;
float rotationsRps;
int meanDist;
bool turning = false;

//Function for working out rotations per second
int counted = 0;
int countedR = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //Begin the serial monitor at 115200
  bluetooth.begin(115200); //Begin the bluetooth serial at 115200
  
  pinMode(trigPinF, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinF, INPUT); // Sets the echoPin as an Input  
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(echoPinB, INPUT);
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

  whatWay();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(stopped)
  {
    //Ping front sensor
    int distanceF = pingF();
    
    ////    Serial    ////
    //Print distance moved since last check
    Serial.print("Distance moved from last stop: ");
    Serial.println(meanDist);
    
    //Print rotation
    if(yaw <=0)
    {
      yaw = 10.5 + yaw;
    }
    else
    {
      yaw = yaw - 10.5;
    }
    
    int roundedYaw = round(yaw);
    Serial.print("Current rotation: ");
    Serial.println(roundedYaw);

    //Print distance from front sensor
    Serial.print("Distance from front sensor: ");
    Serial.println(distanceF);
    
    ////   Bluetooth   ////
    //Print distance moved since last check
    //bluetooth.print("Distance moved from last stop: ");
    bluetooth.println(meanDist);
    //bluetooth.print("\n");
      
    //Print rotation
    //bluetooth.print("Current rotation: ");
    bluetooth.println(roundedYaw);
    //bluetooth.print("\n");
    
    //Print distance from front sensor
    //bluetooth.print("Distance from front sensor: ");
    bluetooth.println(distanceF);
    //bluetooth.print("\n");

    stopped = false;
  }
  else
  {   
    if(!turning)
    {
      int distanceF = pingF();

      //Get a time to return a value every second
      currenttime = millis();
      timer = currenttime;

      //unsigned long thistime = currenttime - timer;
      while(currenttime - timer < 500)
      {
        forward();
        RPS();
        
        int distFromObj = pingF();
        if(distFromObj <= 20)
        {
          float yawNow = yaw;
          while(yaw >= yawNow - 90)
          {
            right();
          }
        }
        
        currenttime = millis();
      }

      float rotationsLps = counterL/20;
      float rotationsRps = counterR/20;

      float disL = distL(rotationsLps);
      float disR = distR(rotationsRps);

      meanDist = round((disL + disR) / 2);

      counterL = 0;
      counterR = 0;
      
      /*Serial.println("Turning right");
      
      right();
      
      Serial.println("Turning left");
      
      left();

      Serial.println("Stopping");*/

      stopped = true;
    }
    else
    {
      
    }

    //delay((increasetime*1000) - (millis() - currenttime));
  }
}

      /*if(distanceF <= 10)
      {
        //Read normalised values
        Vector normalisedvalues = mpu.readNormalizeGyro();
      
        //Calculate yaw
        yaw = yaw + normalisedvalues.XAxis * increasetime;
        
        stopped = true;
      }
      //else
      //{
        //forward();
      //}*/

//Front sensor
int pingF()
{
  digitalWrite(trigPinF, LOW);
  
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPinF, HIGH);
  
  delayMicroseconds(10);
  
  digitalWrite(trigPinF, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPinF, HIGH);
  
  // Calculating the distance
  int distance = duration*0.034/2;
  return distance;
}

int pingL()
{
  digitalWrite(trigPinL, LOW);

  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPinL, LOW);
  long duration = pulseIn(echoPinL, HIGH);

  int distance = duration*0.034/2;
  return distance;
}

int pingR()
{
  digitalWrite(trigPinR, LOW);

  delayMicroseconds(2);
  digitalWrite(trigPinR, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPinR, LOW);
  long duration = pulseIn(echoPinR, HIGH);

  int distance = duration*0.034/2;
  return distance;
}

int pingB()
{
  digitalWrite(trigPinB, LOW);

  delayMicroseconds(2);
  digitalWrite(trigPinB, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPinB, LOW);
  long duration = pulseIn(echoPinB, HIGH);

  int distance = duration*0.034/2;
  return distance;
}

void RPS()
{
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
}

float distL(float rotationsLps)
{
  float distanceL = (circumference*rotationsLps);

  return distanceL;
}

float distR(float rotationsRps)
{
  float distanceR = (circumference*rotationsRps);
  
  return distanceR;
}

//Functions to increment the steps counted with each motor turn
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

int whatWay()
{
  int whatOne = 1;
  int distF = pingF();
  int closest = distF;
  int distB = pingB();
  Serial.println(closest);
  Serial.println(whatOne);

  if (distB < closest)
  {
    closest = distB;
    whatOne = 2;
  }
  Serial.println(closest);
  Serial.println(whatOne);
  
  int distL = pingL();

  if (distL < closest)
  {
    closest = distL;
    whatOne = 3;
  }
  Serial.println(closest);
  Serial.println(whatOne);
  
  int distR = pingR();

  if (distR < closest)
  {
    closest = distR;
    whatOne = 4;
  }

  Serial.println(closest);
  Serial.println(whatOne);

  bool needMove = false;
  
  if(closest > 20)
  {
    //move toward closest
    needMove = true;
  }
  
  switch(whatOne)
  {
    case 1:
    if(needMove)
    {
      while(closest >= 20)
      {
        forward();
        closest = pingF();
      }
      while(yaw >= -90)
      {
        right();
        Serial.println(yaw);
      }
    }
    
    break;
    
    case 2:
    if(needMove)
    {
      while(closest >= 20)
      {
        backward();
        closest = pingB();
      }
      while(yaw >= -90)
      {
        right();
        Serial.println(yaw);
      }
    }
    
    break;
    
    case 3:    
    if(needMove)
    {
      while(yaw <= 90)
      {
        left();
        Serial.println(yaw);
      }
      int getDistance = pingF();
      while(getDistance >= 20)
      {
        forward();
        getDistance = pingF();
      }
      while(yaw >= -90)
      {
        right();
        Serial.println(yaw);
      }
    }
    
    break;
    
    case 4:   
    if(needMove)
    {
      Serial.println("Outside loop ran");
      while(yaw >= -90)
      {
        Serial.println("Inside loop ran");
        right();
        Serial.println(yaw);
      }
      int getDistance = pingF();
      while(getDistance >= 20)
      {
        forward();
        getDistance = pingF();
      }
      while(yaw >= -90)
      {
        right();
        Serial.println(yaw);
      }
    }
    
    break;
  } 
} 

//Functions for movement of the robot
void forward()
{
  int cutime = millis();
  int timeF = cutime;
  
  while(cutime - timeF < 50)
    {
      digitalWrite(leftF, HIGH);
      digitalWrite(rightF, HIGH);
    
      RPS();
    
      cutime = millis();
    }
  //delay(50);
  
  cutime = millis();
  timeF = cutime;
  
  while(cutime - timeF < 50)
    {
      digitalWrite(leftF, LOW);
      digitalWrite(rightF, LOW);
    
      cutime = millis();
    }

  //delay(50);

  Serial.println("Forward");
}

void right()
{
  int cutime = millis();
  int timeF = cutime;
  
  while(cutime - timeF < 500)
  {
    cutime = millis();
    
    digitalWrite(leftB, HIGH);

    Vector normalisedvalues = mpu.readNormalizeGyro();
      
    //Calculate yaw
    yaw = yaw + normalisedvalues.XAxis * increasetime;

    delay((increasetime*1000) - (millis() - cutime));
  }
  
  digitalWrite(leftB, LOW);
}

void left()
{
  int cutime = millis();
  int timeF = cutime;
  
  while(cutime - timeF < 500)
  {
    cutime = millis();
    
    digitalWrite(rightB, HIGH);

    Vector normalisedvalues = mpu.readNormalizeGyro();
      
    //Calculate yaw
    yaw = yaw + normalisedvalues.XAxis * increasetime;

    delay((increasetime*1000) - (millis() - cutime));
  }

  digitalWrite(rightB, LOW);
}

void backward()
{
  digitalWrite(leftF, LOW);
  digitalWrite(leftB, HIGH);
  digitalWrite(rightF, LOW);
  digitalWrite(rightB, HIGH);
}
