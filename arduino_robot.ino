#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

//Set up the serial input/output.
SoftwareSerial bluetooth(8,7);

//Set the magentometer ready for use.
MPU6050 mpu;

//Pins for connected sensors.
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

//Variables required for calculations.
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

//Function for working out rotations per second.
int counted = 0;
int countedR = 0;

void setup() {
	Serial.begin(115200); //Begin the serial monitor at 115200.
	bluetooth.begin(115200); //Begin the bluetooth serial at 115200.

	pinMode(trigPinF, OUTPUT); // Sets the trigPin as an Output.
	pinMode(echoPinF, INPUT); // Sets the echoPin as an Input.
	pinMode(trigPinL, OUTPUT);
	pinMode(echoPinL, INPUT);
	pinMode(trigPinR, OUTPUT);
	pinMode(echoPinR, INPUT);
	pinMode(trigPinB, OUTPUT);
	pinMode(echoPinB, INPUT);
	pinMode(leftF, OUTPUT); //Set pins for motors as an output.
	pinMode(leftB, OUTPUT);
	pinMode(rightF, OUTPUT);
	pinMode(rightB, OUTPUT);
	pinMode(encoderInL, INPUT); //Set pins for wheel encoders.
	pinMode(encoderInR, INPUT); //as input.

	//FOR SETTING UP MPU ONLY.
	while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
		Serial.println("Could not find a valud MPU6050 sensor");
		delay(500);
	}
	
	//Get the initial direction the robot is facing.
	mpu.calibrateGyro();
	
	//Find closest wall.
	whatWay();
}

void loop() 
{
	//This if else could be replaced by as it is somewhat redundant, becomes clear why in the else.
	if(stopped)
	{
		//Stop movement
		analogWrite(rightF, 0);
		analogWrite(leftF, 0);
		analogWrite(rightB, 0);
		analogWrite(leftB, 0);

		//Ping sensors
		int distanceF = pingF();
		int distanceR = pingR();
		int distanceB = pingB();
		int distanceL = pingL();

		/*
		//THIS IS FOR TESTING PURPOSES
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
		*/

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
		//Setup distances.
		int distanceF = pingF();
		int currentDist = pingL();

		//Get a time to return a value every second.
		currenttime = millis();
		timer = currenttime;

		//Execute every 0.5 seconds.
		while (currenttime - timer < 500)
		{
			//Set lowest distance to left wall recorded.
			int lowestval = pingL();
			int newval;
			float goodYaw;

			//Check to see if lowestval has changed by 5cm up or down in the past 0.5s time frame, if not proceed forward.
			//This is for turning right (robot too close to wall).
			if (lowestval <= currentDist - 5)
			{
				//Setup newval, keep turning right until value stops getting lower.
				newval = pingL();
				while (newval <= lowestval)
				{
					right();
					newval = pingL();

					//Set new lowestval and set goodYaw as the yaw at the lowest value.
					if (newval < lowestval)
					{
						lowestval = newval;
						goodYaw = yaw;
					}
				}

				//Turn left until back at the goodYaw.
				while (yaw > goodYaw)
				{
					left();
				}
			}
			//This is for turning left (robot too far from wall).
			else if (lowestval >= currentDist + 5)
			{
				//Setup newval, keep turning left until value stops getting lower.
				newval = pingL();
				while (newval <= lowestval)
				{
					left();
					newval = pingL();

					//Set new lowestval and set goodYaw as the yaw at the lowest value.
					if (newval < lowestval)
					{
					lowestval = newval;
					goodYaw = yaw;
					}
				}

				//Turn left until back at the goodYaw.
				while (yaw > goodYaw)
				{
					right();
				}
			}
			else
			{
				forward();
			}

			RPS();

			//Setup distance from front sensor.
			int distFromObj = pingF();

			//If front sensor pings 30cm or lower turn 90 degrees right.
			if (distFromObj <= 30)
			{
				float yawNow = yaw;
				while (yaw >= yawNow - 89)
			{
				right();
			}
			//Stop back motor and setup current left distance for next run of loop.
			analogWrite(rightB, 0);
			currentDist = pingL();
			}
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

//Left sensor
int pingL()
{
	digitalWrite(trigPinL, HIGH);

	delayMicroseconds(10);

	digitalWrite(trigPinL, LOW);
	long duration = pulseIn(echoPinL, HIGH);

	int distance = duration*0.034/2;
	return distance;
}

//Right sensor
int pingR()
{
	digitalWrite(trigPinR, HIGH);

	delayMicroseconds(10);

	digitalWrite(trigPinR, LOW);
	long duration = pulseIn(echoPinR, HIGH);

	int distance = duration*0.034/2;
	return distance;
}

//Back sensor
int pingB()
{
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
	//Setup values, set front sensor as lowest value.
	int whatOne = 1;
	int distF = pingF();
	int closest = distF;
	int distB = pingB();
	float yawNow;
	bool needMove = false;

	//Check if back sensor is closer than front sensor, if yes set as closest value.
	if (distB < closest)
	{
		closest = distB;
		whatOne = 2;
	}

	int distL = pingL();

	//Check if left sensor is closer than front sensor, if yes set as closest value.
	if (distL < closest)
	{
		closest = distL;
		whatOne = 3;
	}

	int distR = pingR();

	//Check if right sensor is closer than front sensor, if yes set as closest value.
	if (distR < closest)
	{
		closest = distR;
		whatOne = 4;
	}

	//If closest > 30 then move toward closest
	if (closest > 30)
	{
		needMove = true;
	}

	if (needMove)
	{
		switch (whatOne)
		{
		//Forward
		case 1:
			//Forward until front sensor < 30
			while (closest >= 30)
			{
				forward();
				closest = pingF();
			}

			//Turn right so that close wall is on the left.
			yawNow = yaw;
			while (yaw >= yawNow - 89)
			{
				right();
			}

			//Turn off motors.
			analogWrite(leftB, 0);
			analogWrite(rightB, 0);

			break;

		//Back
		case 2:
			//Backwards until back sensor < 30
			while (closest >= 30)
			{
				backward();
				closest = pingB();
			}

			//Turn left so that close wall is on the left.
			yawNow = yaw;
			while (yaw >= yawNow + 89)
			{
				left();
			}

			//Turn off motors.
			analogWrite(leftB, 0);
			analogWrite(rightB, 0);

			break;

		//Left
		case 3:
			//Turn left so that close wall is infront.
			yawNow = yaw;
			while (yaw >= yawNow + 89)
			{
				left();
			}

			//Forward until front sensor < 30
			int getDistance = pingF();
			while (getDistance >= 30)
			{
				forward();
				getDistance = pingF();
			}

			//Turn right so that close wall is on the left.
			yawNow = yaw;
			while (yaw >= yawNow - 89)
			{
				right();
			}

			//Turn off motor.
			analogWrite(leftB, 0);

			break;

		//Right
		case 4:
			//Turn right so that close wall is infront.
			yawNow = yaw;
			while (yaw >= yawNow - 89)
			{
				right();
			}

			//Forward until front sensor < 30
			int getDistance = pingF();
			while (getDistance >= 30)
			{
				forward();
				getDistance = pingF();
			}

			//Turn right so that close wall is on the left.
			yawNow = yaw;
			while (yaw >= yawNow - 89)
			{
				right();
			}

			//Turn off motor.
			analogWrite(rightB, 0);

			break;
		}
	}
}

//Functions for movement of the robot
void forward()
{
	analogWrite(leftB,  0);
	analogWrite(rightB, 0);

	//Different values to conteract the imbalance in the motors. These can be changed, higher = faster.
	analogWrite(leftF,  125);
	analogWrite(rightF, 117);

	RPS();
}

void right()
{
	currenttime = millis(); 
	Vector normalisedvalues = mpu.readNormalizeGyro();

	analogWrite(leftB, 0);
	analogWrite(rightF,  0);
	analogWrite(leftF,  0);

	analogWrite(rightB, 122);

	yaw = yaw + normalisedvalues.XAxis * increasetime;

	if(millis() - currenttime > 9)
	{
	delay((increasetime*2000) - (millis() - currenttime));
	}
	else
	{
	delay((increasetime*1000) - (millis() - currenttime));
	}
}

void left()
{
	currenttime = millis(); 
	Vector normalisedvalues = mpu.readNormalizeGyro();

	analogWrite(rightB,  0);
	analogWrite(rightF,  0);
	analogWrite(leftF,  0);

	analogWrite(leftB, 130);

	yaw = yaw + normalisedvalues.XAxis * increasetime;

	if(millis() - currenttime > 9)
	{
	delay((increasetime*2000) - (millis() - currenttime));
	}
	else
	{
	delay((increasetime*1000) - (millis() - currenttime));
	}
}

void backward()
{
	analogWrite(leftF,  0);
	analogWrite(rightF, 0);

	analogWrite(leftB,  125);
	analogWrite(rightB, 117);

	RPS();
}
