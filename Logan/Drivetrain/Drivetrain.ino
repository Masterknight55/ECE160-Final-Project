
/* -------------------------------------------------------------------------- */
/*                                 Drive Train                                */
/* -------------------------------------------------------------------------- */

#include <Servo.h>


Servo servoLeft;                                   
Servo servoRight;
Servo servoGripper;  


/* --------------------------------- Pinout --------------------------------- */

const int servoLeftPin = 13;                       
const int servoRightPin = 11;
const int servoGripperPin = 10;

/** This is the global value for the servo code
 * by setting this global varible in our code 
 * for the servo power we won't deal with as much conflicts
 * with setting the servo. 
 */
double servoLeftPositionValue = 1500;
double servoRightPositionValue = 1500; 

/* -------------------------------------------------------------------------- */


void setup() {


  // put your setup code here, to run once:
  setupServos();
}


void loop()
{
  // put your main code here, to run repeatedly:
  setServos();

}

/* -------------------------------------------------------------------------- */
/*                            Drive Train Movement                            */
/* -------------------------------------------------------------------------- */
/** This section of code uses the write microseconds commands to control how 
 * the robot moves.
 */


/** This function actually sets the servo positions and is all that needs 
 * to be ran in the loop function
 */
void setServos()
{
servoLeft.writeMicroseconds(servoLeftPositionValue);
servoRight.writeMicroseconds(servoRightPositionValue);
}

void setupServos()
{
  servoLeft.attach(servoLeftPin);                  
  servoRight.attach(servoRightPin);                
  servoTurret.attach(servoGripperPin);              
}


/* -------------------------- Servo Motor Commands -------------------------- */
/** 
 * Servo Motor Still Command: 1500 us
 * Servo Motor Turn Clockwise: 1300 us
 * Servo Motor Turns Counter Clockwise: 1700 us
 * 
 * By looking at these values we can deterime these ranges for the servo values
 * 
 * 1300 - 1500 for turning clockwise
 * 
 * 1500 - 1700 for turing counter clock wise
 * 
 * This means we can use the map function used in the pervious mini projects 
 * to map the left and the right motor to the left and right joystick. 
 * 
 */
/* -------------------------------------------------------------------------- */


/**This will be the main function used to control the drive train. The left
 * and the right parameters will be used with the controllers left and right
 * analog sticks. 
 */


/* -------------------------- Manual Drive Function ------------------------- */

//TODO Figure out the values for the analog stick. Righ now they are at -100 and 100 
void driveTrainManualDrive(double left, double right)
{

//Left Logic
if(left == 0)
{

servoLeftPositionValue = 1500;

}


//This case means it is positive and will use the clockwise map  
else if(left > 0)
{

servoLeftPositionValue = map(left, 0, 100, 1500, 1300);

}

//This case measn it is negetive and will use the counter clockwise map
else if(left < 0)
{

servoLeftPositionValue = map(right, -100, 0, 1500, 1700);

}

//Left Logic
if(right == 0)
{

servoRightPositionValue = 1500;

}


//This case means it is positive and will use the clockwise map  
else if(right > 0)
{

servoRightPositionValue = map(left, 0, 100, 1500, 1300);

}

//This case measn it is negetive and will use the counter clockwise map
else if(right < 0)
{

servoRightPositionValue = map(right, -100, 0, 1500, 1700);

}

/* -------------------------------------------------------------------------- */


}

/* ----------------------------- Drive Commands ----------------------------- */
/** These are the different commands you can give the Robots drivetrain. 
 * These commands could be useful as auto actions for in the auto modes.
 */
void driveTrainStop()
{
  
double servoLeftPositionValue = 1500;
double servoRightPositionValue = 1500;

} 

//Clockwise is forwards
void driveTrainForward()
{

double servoLeftPositionValue = 1300;
double servoRightPositionValue = 1300;

}

//Conuter clockwise is reverse
void driveTrainReverse()
{

double servoLeftPositionValue = 1700;
double servoRightPositionValue = 1700;

}

void driveTrainSpinLeft()
{

double servoLeftPositionValue = 1700;
double servoRightPositionValue = 1300;

}

void driveTrainSpinRight()
{

double servoLeftPositionValue = 1300;
double servoRightPositionValue = 1700;

}

void driveTrainTurnLeft()
{

double servoLeftPositionValue = 1500;
double servoRightPositionValue = 1300;

}

void driveTrainTurnRight()
{

double servoLeftPositionValue = 1700;
double servoRightPositionValue = 1500;

}

/* -------------------------------------------------------------------------- */


/* ---------------------------- Gripper Commands ---------------------------- */
/** This Section of code controls the different states of the servo */
void gripperClose()
{
servo.write(0);
}


void gripperCenterPosition()
{
servo.write(90);
}


void gripperOpen()
{
servo.write(180);
}

/* -------------------------------------------------------------------------- */


