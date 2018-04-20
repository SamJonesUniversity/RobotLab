#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <Stepper.h>

//Set up the serial input/output
SoftwareSerial bluetooth(8,7);

//Set the magentometer ready for use
MPU6050 mpu;

//Pins for connected sensors
const int trigPinF = 13;
const int echoPinF = 12;
const int trigPinL = 10;
const int echoPinL = 11;
const int trigPinR = A2;
const int echoPinR = A3;
const int trigPinB = A1;
const int echoPinB = A0;
const int encoderInL = 2;
const int encoderInR = 4;
const int leftF = 6;
const int leftB = 9;
const int rightF = 3;
const int rightB = 5;

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

//Variables used to calculate if the robot is travelling too far from the wall while moving forward, drifing solution
bool firstRun = true;
int lastDist = 0;

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
    //Ping sensors
    int distanceF = pingF();
    int distanceR = pingR();
    int distanceB = pingB();
    int distanceL = pingL();
    
    ////    Serial    ////
    //Print distance moved since last check
    Serial.print("Distance moved from last stop: ");
    Serial.println(meanDist);
    
    int roundedYaw = round(yaw);
    Serial.print("Current rotation: ");
    Serial.println(roundedYaw);

    //Print distance from front sensor
    Serial.print("Distance from front sensor: ");
    Serial.println(distanceF);
    Serial.print("Distance from right sensor: ");
    Serial.println(distanceR);
    Serial.print("Distance from back sensor: ");
    Serial.println(distanceB);
    Serial.print("Distance from left sensor: ");
    Serial.println(distanceL);
    
    ////   Bluetooth   ////
    //Print distance moved since last check
    bluetooth.println(meanDist);
    bluetooth.print("\n");
      
    //Print rotation
    bluetooth.println(roundedYaw);
    bluetooth.print("\n");
    
    //Print distance from sensors
    bluetooth.println(distanceF);
    bluetooth.print("\n");
    bluetooth.println(distanceR);
    bluetooth.print("\n");
    bluetooth.println(distanceB);
    bluetooth.print("\n");
    bluetooth.print("Distance left: ");
    bluetooth.println(distanceL);
    bluetooth.print("\n");

    stopped = false;
  }
  else
  {   
    int distanceF = pingF();
    
    //Get a time to return a value every second
    currenttime = millis();
    timer = currenttime;
  
    //unsigned long thistime = currenttime - timer;
    while(currenttime - timer < 500)
    {
      int currentDist = pingL();
      /*if(firstRun = true)
      {
        lastDist = currentDist;
        firstRun = false;
      }*/
      
      Serial.print("This is the current left sensor distance: ");
      Serial.println(currentDist);
      
      if(currentDist < 30)
      {
        analogWrite(rightF, 132);
        analogWrite(leftF, 170);
      }
      else if (currentDist > 30)
      {
        analogWrite(rightF, 162);
        analogWrite(leftF, 140);
      }
      else
      {
        forward();
      }
      
      RPS();
      
      int distFromObj = pingF();
      Serial.print("Distance from front ");
      Serial.println(distFromObj);
      if(distFromObj <= 30)
      {
        float yawNow = yaw;
        while(yaw >= yawNow - 89)
        {
          Serial.println(yawNow);
          Serial.println(yaw);
          right();
        }
        firstRun = true;
        analogWrite(rightB, 0);
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
  
    stopped = true;
  }
}

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

void aligned()
{
  int distL = pingL();
  int lowest = distL;

  distL = pingL();
  while(pingL < lowest)
  {
    if(distL < lowest)
    {
      lowest = distL;
    }
    right();
  }
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
  
  if(closest > 30)
  {
    //move toward closest
    needMove = true;
  }
  
  switch(whatOne)
  {
    //Forward
    case 1:
    if(needMove)
    {
      while(closest >= 30)
      {
        forward();
        closest = pingF();
      }
      while(yaw >= -89)
      {
        right();
        Serial.println(yaw);
      }
      
      analogWrite(leftB, 0);
      analogWrite(rightB, 0);
    }
    
    break;

    //Back
    case 2:
    if(needMove)
    {
      while(closest >= 30)
      {
        backward();
        closest = pingB();
      }
      while(yaw >= -89)
      {
        right();
        Serial.println(yaw);
      }

      analogWrite(leftB, 0);
      analogWrite(rightB, 0);
    }
    
    break;

    //Left
    case 3:    
    if(needMove)
    {
      while(yaw <= 89)
      {
        left();
        Serial.println(yaw);
      }
      int getDistance = pingF();
      while(getDistance >= 30)
      {
        forward();
        getDistance = pingF();
      }
      while(yaw >= -89)
      {
        right();
        Serial.println(yaw);
      }
      analogWrite(leftB, 0);
    }
    
    break;

    //Right
    case 4:   
    if(needMove)
    {
      while(yaw >= -89)
      {
        right();
        Serial.println(yaw);
      }
      int getDistance = pingF();
      while(getDistance >= 30)
      {
        forward();
        getDistance = pingF();
      }
      while(yaw >= -89)
      {
        right();
        Serial.println(yaw);
      }

      analogWrite(rightB, 0);
    }
    
    break;
  } 
} 

//Functions for movement of the robot
void forward()
{
  analogWrite(leftB,  0);
  analogWrite(rightB, 0);
  
  analogWrite(leftF,  125);
  analogWrite(rightF, 117);

  RPS();

  //analogWrite(leftF,  0);
  //analogWrite(rightF, 0);
  
  Serial.println("Forward");
}

void right()
{
  int currenttime = millis();
  
  Vector normalisedvalues = mpu.readNormalizeGyro();
  
  analogWrite(leftB, 0);
  analogWrite(rightF,  0);
  analogWrite(leftF,  0);
    
  analogWrite(rightB, 117);

  //Calculate yaw
  yaw = yaw + normalisedvalues.XAxis * increasetime;

  delay((increasetime*1000) - (millis() - currenttime));
}

void left()
{
  int currenttime = millis();
  
  Vector normalisedvalues = mpu.readNormalizeGyro();
  
  analogWrite(rightB,  0);
  analogWrite(rightF,  0);
  analogWrite(leftF,  0);
    
  analogWrite(leftB, 125);
  //Calculate yaw
  yaw = yaw + normalisedvalues.XAxis * increasetime;

  delay((increasetime*1000) - (millis() - currenttime));
}

void backward()
{
  analogWrite(leftF,  0);
  analogWrite(rightF, 0);
  
  analogWrite(leftB,  125);
  analogWrite(rightB, 117);

  RPS();
  
  Serial.println("Backward");
}
