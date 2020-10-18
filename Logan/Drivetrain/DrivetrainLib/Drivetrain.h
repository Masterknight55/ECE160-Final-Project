/* -------------------------------------------------------------------------- */
/*                              Drivetrain Header                             */
/* -------------------------------------------------------------------------- */

#ifndef Drivetrain_h
#define Drivetrain_h

#include "Ardunio.h"

class Drivetrain
{

    public:
        Drivetrain(int leftMotorPin, int rightMotorPin, int gripperPin);
        void tankMovement(double leftMotorPower, double rightMotorPower);
        void stop();
        void setServos();
        void closeGripper();
        void openGripper();
        void centerGripper();
    
    private:
        double servoLeftPositionValue = 1500;
        double servoRightPositionValue = 1500;
        int    servoGripperPositionValue = 90;
        
}