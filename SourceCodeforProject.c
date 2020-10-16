/*ShotBot(TM) source code

Authors: Balen Seeton, Neil Fernandes, Abhimanyu Singh, Ansh Sahny - University of Waterloo

*/

//Global Constants
#include "PC.FileIO.c"
#include "EV3Servo-lib-UW.c"

const int MAX_SHOTS_TAKEN = 75; //Records for FGA in a game / 48* 30
const int ZONES = 3;

const int NUM_RED = 7;
const int NUM_BLUE = 5;
const int NUM_YELLOW = 2;

const int RED_POINTS[2][NUM_RED] = {/*Insert points for red portion*/ };
const int BLUE_POINTS[2][NUM_BLUE] = {/* Insert points for blue portion*/ };
const int YELLOW_POINTS[2][NUM_YELLOW] = {/* Insert points for yellow portion*/ };

const float INITIAL_VELOCITY = 18.55;

//Set timer function

float setTimer() {
    int count = 0;
	while (count >= 0 && count <= 30)
	{
		displayString(7, "Shooting Time");
		displayString(8, "%d:00", count);

		while (!getButtonPress(buttonAny))
		{ }

		if (getButtonPress(buttonLeft) {
			while(getButtonPress(buttonLeft))
			{ }
			count--;
		}
		else if (getButtonPress(buttonRight)) {
			while(getButtonPress(buttonRight))
			{ }
			count ++;
		}
		else if (getButtonPress(buttonUp)) {
			while(getButtonPress(buttonUp))
			{ }

			count += 5;
		}
		else if (getButtonPress(buttonDown)) {
			while(getButtonPress(buttonDown))
			{ }
			count-=5;

		}
		else {
			break;
		}

	}
	return count;
}

//calcAngle, calcDistance and calcVertAngle functions for robot kinematics parameters

int calcHorAngle(float xDist, float yDist) {
	int angle = 0;
	angle = atan2(30 - yDist, -(20 - xDist)) * 180 / PI;
	if (angle < 90) {
		angle = angle - 180;
	}

	return (int)angle;
}

float calcDistance(float xDist, float yDist) {
	float dist = 0;
	dist = sqrt(pow((30 - yDist), 2) + pow((20 - xDist), 2));
	return dist;
}

float calcVerAngle(float xDist, float yDist) {
	int theta = 0;
	float distance = calcDistance(xDist, yDist);
	
	float arcsinInput = pow(INITIAL_VELOCITY, 2.0) / (9.8 * distance);
	theta = asin(arcsinInput) * 180 / PI;
	return theta;
}

//shotMechanics function for the shooting mechanism
void shotMechanics(float xDist, float yDist) {
	float horAngle = calcHorAngle(xDist, yDist);
	float vertAngle = calcVertAngle(xDist, yDist);

	setServoPosition(S4, 1/*Servo Number*/, -10);
	setServoPosition(S4, 2/*Servo Number*/, 10);

	motor[motorC] = 100;
	time1[T1] = 0;
	while (time1[T1] < 3700) 
	{ }
	motor[motorC] = 0; //Launch point

	setServoPostion(S4, 1, 0);
	setServoPostion(S4, 2, -45);
}

int roundingAlg(float number) {
	float smallerDifference = ZONES * MAX_SHOTS_TAKEN; //Max Difference Possible
	int closestInt = 0;
	int i = 1;

	for (auto i : ZONES) {
		if (fabs(number - i) < smallerDifference) {
			smallerDifference = fabs(number - i);
			closestInt = i;
		}
	}

	return closestInt; //Closest interger to rounding from 1 to 3
}

//makeDescision function

int makedDecision(int numOfShots, float& currentPps, float desired Pps) {
	float decisonValue = 0;
	int colourDecision = 0;

	decisonValue = desiredPps * (numShots + 1) - currentPps * (numOfShots);// Decsion making algorithm
	displayString(4, "Decision Value: %f", decisonValue);
	displayString(5, "DesiredPPS: %f", desiredPps);
	wait1Msec(5000);

	colourDecision = roundingAlg(decisonValue);

	return colourDecision;
}

//movetoPos function for the robot to move to a certain postion coordinate

void moveToPos(int cDecision, float xDist, float yDist, float& xDecision, float& yDecision) {
	int pDecision = 0;

	if (cDecision == 3) {
		pDecision = random(NUM_RED - 1);
		xDecision = RED_POINTS[0][pDecision];
		yDecision = RED_POINTS[1][pDecision];
	}
	else if (cDecision == 2){
		pDecision = random(NUM_BLUE - 1);
		xDecision = BLUE_POINTS[0][pDecision];
		yDecision = BLUE_POINTS[1][pDecision];
	}
	else {
		pDecision = random(NUM_YELLOW - 1);
		xDecision = YELLOW_POINTS[0][pDecision];
		yDecision = YELLOW_POINTS[1][pDecision];
	}

	if (xDecision > xDist) {
		motor[motorA] = 25; //Moves motor in the x distance
		while (SensorValue[S1] < xDecision)
		{}
	}
	else {
		motor[motorA] = -25; //Moves motor in the -x direction
		while (SensorValue[S1] > xDecision)
		{ }
	}
	motor[motorA] = 0;

	if (yDecision > yDist) {
		motor[motorB] = -25 //Moves motor in the -y direction
		while(SensorValie[S2] < yDecision)
		{ }

	}
	else {
		motor[motorB] = 25;// Moves motor in the y direction
		while (SensorValie[S2] > yDecision)
		{}
	}
	motor[motorB] = 0;
}

//decisionMaking algorithm that brings the ShotBot to life
void decisionMaking() {
	TFileHandle inFile;
	bool fileOkay = openReadPC(inFile, "shotData.txt");

	if (!fileOkay)
	{
		displayString(5, "Error - File not available");
		wait1Msec(5000);
	}
	else {
		//Intilization of parameters

		int i = 0;
		int shotResult[MAX_SHOTS_TAKEN];
		int shotPoints[MAX_SHOTS_TAKEN];

		for (int i = 0; i < MAX_SHOTS_TAKEN; i++) {
			shotResult[i] = 0;
			shotPoints[i] = 0;
		}
		//Initilization ends

		//Reading in of the file starts
		int time = 0;
		int shotsTaken = 0;
		int pointsScored = 0;
		float pointsPerShot = 0;
		readIntPC(inFile, time);

		i = 0;

		while (readIntPC(inFile, shotResult[i]) && readIntPC(inFile, shotPoints[i])) {
			if (shotResult[i] == 1) {
				pointsScored += shotPoints[i];
			}
			if (i > MAX_SHOTS_TAKEN) {
				break;
			}

			i++;
		}
		shotsTaken = i + 1;
		pointsPerShot = (float)pointsScored / (float)shotsTaken;
		// Reading in the file ends

		//Decision making process starts

		i = 0;
		int finalDecision = 0;
		int robotPoints = 0;
		float robotPointsPerShot = 0;
		int shotsMade = 0;

		float count = setTimer();
		time1[T2] = 0;
		while (time1[T2] < count * 60 * 1000) {

			float xDec = 0;
			float yDec = 0;

			moveToPos(finalDecision, SensorValue[S1/*Ultrasonics 1*/], SensorValue[S2/*Ultrasonics 2*/, xDec, yDec]);
			shotMechanics(xDec, yDec);
			time1[T1] = 0;
			while ((SensorValue[S3] == 0) && (time1[T1] < 2000))
			{ }

			if (time1[T1] < 1000) {
				shotsMade++;

				if (SensorValue[S2] < 11) {
					robotPoints += 3;
				}
				else if ((SensorValue[S2] >= 15.5 && SensorValue[S2] < 18.5) && (SensorValue[S1] >= 15.5 && SensorValue[S1] < 24.5) {
					robotPoints +=1;
				}
				else {
					robotPoints +=2;
				}
			}

			robotPointsPerShot= (float)robotPoints/(i+1);
			finalDecision = makedDecision(i, robotPointsPerShot, pointsPerShot);
				i++;
		}
		float shotPercentage = (shotsMade / i + 1) * 100;
		displayBigTextLine(4, "SHOTBOT Shot Percentage: ");
		displayBigTextLine(6, "%f%%", shotPercentage);

		while(!getButtonPress(buttonAny))
		{ }

		while(getButtonPress(buttonEnter))
		{ }
	}
}

//mainFuction implementation for the ShotBot

task main() {
	SensorType[S1] = sensorEV3_Ultrasonic;
	wait1Msec(100);
	SensorType[S2] = sensorEV3_Ultrasonic;
	wait1Msec(100);
	SenorType[S3] = sensorEV3_Touch;
	wait1Msec(100);
	SensorType[S4] = sensorI2CCustom9V; //Vertical and horizontal movement for the robot
	wait1Msec(100);
	
	decisionMaking();
}