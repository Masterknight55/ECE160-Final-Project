
#include <Servo.h>

Servo servoLeft;                                   // Servo object instances
Servo servoRight;
Servo servoGripper;  


const int servoLeftPin = 13;                       // I/O Pin constants
const int servoRightPin = 12;
const int servoGripperPin = 11;

/** This is the global value for the servo code
 * by setting this global varible in our code 
 * for the servo power we won't deal with as much conflicts
 * with setting the servo. 
 */
double servoLeftPositionValue = 1500;
double servoRightPositionValue = 1500; 

void setup() {


  // put your setup code here, to run once:
  servoLeft.attach(servoLeftPin);                  
  servoRight.attach(servoRightPin);                
  servoTurret.attach(servoGripperPin);              
}


void loop()
{
  // put your main code here, to run repeatedly:


}

/* -------------------------------------------------------------------------- */
/*                            Drive Train Movement                            */
/* -------------------------------------------------------------------------- */
/** This section of code uses the write microseconds commands to control how 
 * the robot moves.
 */


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
 * This means we can use the map function used i nthe pervious mini projects 
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

servoLeftPositionValue = map(left, 0, 100, 1300, 1500);

}

//This case measn it is negetive and will use the counter clockwise map
else if(left < 0)
{

servoLeftPositionValue = map(right, -100, 0, 1500, 1700);

}

/* -------------------------------------------------------------------------- */


}

/* --------------------------- Auto Drive Commands -------------------------- */

void driveTrainStop()
{

} 

void driveTrainForward()
{

}

void driveTrainReverse()
{

}

void driveTrainSpinLeft()
{

}

void driveTrainSpinRight()
{

}

void driveTrainTurnLeft()
{

}

void driveTrainTurnRight()
{

}

/* -------------------------------------------------------------------------- */


/* ---------------------------- Gripper Commands ---------------------------- */
/** This Section of code controls the different states of the servo */
void gripperClose()
{
servo.write(servoGripper, 0);
}


void gripperCenterPosition()
{
servo.write(servoGripper, 90);
}


void gripperOpen()
{
servo.write(servoGripper, 180);
}

/* -------------------------------------------------------------------------- */


