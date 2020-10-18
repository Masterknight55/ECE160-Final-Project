/* -------------------------------------------------------------------------- */
/*                               Drivetrain CPP                               */
/* -------------------------------------------------------------------------- */

#include "Ardunio.h"
#include "Drivetrain.h"
#include "Servo.h"

Servo servoLeft;
Servo servoRight;
Servo servoGripper;

double servoLeftPositionValue = 1500;
double servoRightPositionValue = 1500;
int servoGripperPositionValue = 90;

Drivetrain::Drivetrain(int servoLeftPin, int servoRightPin, int servoGripperPin);
{
    servoLeft.attach(this->servoLeftPin);
    servoRight.attach(this->servoRightPin);
    servoGripper.attach(this->servoGripperPin);
}

void Drivetrain::setServos()
{
    servoLeft.writeMicroseconds(servoLeftPositionValue);
    servoRight.writeMicroseconds(servoRightPositionValue);
}

void Drivetrain::tankMovement(double leftMotorPower, double rightMotorPower)
{
    //Left Logic
    if (leftMotorPower == 0)
    {

        servoLeftPositionValue = 1500;
    }

    //This case means it is positive and will use the clockwise map
    else if (leftMotorPower > 0)
    {

        servoLeftPositionValue = map(leftMotorPower, 0, 100, 1500, 1300);
    }

    //This case measn it is negetive and will use the counter clockwise map
    else if (leftMotorPower < 0)
    {

        servoLeftPositionValue = map(leftMotorPower, -100, 0, 1500, 1700);
    }

    //Left Logic
    if (rightMotorPower == 0)
    {

        servoRightPositionValue = 1500;
    }

    //This case means it is positive and will use the clockwise map
    else if (rightMotorPower > 0)
    {

        servoRightPositionValue = map(rightMotorPower, 0, 100, 1500, 1300);
    }

    //This case measn it is negetive and will use the counter clockwise map
    else if (rightMotorPower < 0)
    {

        servoRightPositionValue = map(rightMotorPower, -100, 0, 1500, 1700);
    }
}

void Drivetrain::stop()
{

    double servoLeftPositionValue = 1500;
    double servoRightPositionValue = 1500;
}

void Drivetrain::openGripper()
{
    servoGripperPositionValue = 180;
}

void Drivetrain::closeGripper()
{
    servoGripperPositionValue = 0;
}

void Drivetrain::centerGripper()
    servoGripperPositionValue = 90;
{
}