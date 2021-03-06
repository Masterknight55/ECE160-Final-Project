/* -------------------------------------------------------------------------- */
/*                         Different Breed Final Code                         */
/* -------------------------------------------------------------------------- */
/** 
 * DifferentBreed_FinalCode.ino
 * Logan Manthey 
 * 11/15/20 
 * 
 * The following code controls a customizable BOEBot which competed in the 
 * Golden Goose Gala challenge. The code is split into 2 main sections 
 * Auto modes and Manual controls. Durring Auto the code allows the 
 * robot to deliver targets to the desginated location specified by 
 * the mode and durring the manual mode the robot is controlled by a 
 * PS2 Controller. 
 * 
 * Main Functions
 * 
 *  Setup Functions
 *    The code uses different functions for each piece of hardware such as 
 *    the controller, and the servos.
 * 
 *  Control Functions
 *    Control Functions take in the sensor input and convert them into 
 *    drivetrain movement. For example the manual control function 
 *    which takes in values from the controller and then makes the 
 *   drive train move depending on them. 
 * 
 *  Calculation Functions
 *    Calculation functions return a double or boolean after taking in 
 *    parameters. For example there is a function which returns a motor
 *    value for the drive train with a given distance value. 
 * 
 *   Auto Actions
 *    These functions use auto booleans to make them only run once. These
 *    actions include opening the gripper and driving with a given parameter
 *    such as time.
 *  
 * Main Variables 
 *  The main variables used throughout the code include a pinout, auto 
 *  action booleans, and sensor values. 
 * 
 * Summary of Hardware used
 *  1 PlayStation Controller receiver and controller 
 *  1 Infrared LED
 *  1 RobotShop Line Following Sensor
 *  1 Sonar Sensor
 *  1 BOEBot Base
 * 
 */
/* -------------------------------------------------------------------------- */


/* --------------------------------- Imports -------------------------------- */

#include <Servo.h>
#include <PS2X_lib.h>
#include <SR04.h>

/* -------------------------------------------------------------------------- */


/* --------------------------------- Objects -------------------------------- */

PS2X ps2x;
Servo servoLeft;
Servo servoRight;
Servo servoGripper;

/* -------------------------------------------------------------------------- */


int error = 0;
byte type = 0;
byte vibrate = 0;

/* ------------------------ Auto Variables and States ----------------------- */

int right = 1;
int left = 2;
int straight = 3;
int previousLineFollowState = 0;

#define MANUAL 0 // defining our states
#define AUTOLEFT 1
#define AUTORIGHT 2
#define AUTOMIDDLE 3
int STATE = MANUAL; // start in the IDLE state

boolean autoActionComplete0 = false;
boolean autoActionComplete1 = false;
boolean autoActionComplete2 = false;
boolean autoActionComplete3 = false;

/* -------------------------------------------------------------------------- */


/* --------------------------------- Pinout --------------------------------- */

const int servoLeftPin = 13;
const int servoRightPin = 12;
const int servoGripperPin = 10;

int rightLightSensorPin = A2;  
int leftLightSensorPin = A1;   
int middleLightSensorPin = A3; 

int frontSonarrPin = 6;
int rightSonarrPin = 8;
int leftSonarrPin = 7;

double whiteLevel = 500;
double blackLevel = 0;

/** This is the global value for the servo code
 * by setting this global varible in our code 
 * for the servo power we won't deal with as much conflicts
 * with setting the servo. 
 */
double servoLeftPositionValue = 1500;
double servoRightPositionValue = 1500;

double analogZero = 523;

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                Main Section                                */
/* -------------------------------------------------------------------------- */

void setup()
{
  setupServos();
  gripperClose();
  controllerSetup();
}

void loop()
{

  controllerLoop();
  printSensorValues();

/** 
 * This Section is the State machine that is running to determine what main mode 
 * the robot is in. These modes are Manual and the three auto modes which are for
 * the left delivery, right delivery, and middle delivery.
 * **/
  switch (STATE)
  {



  case MANUAL:
    
    manualControls();
    break;



  case AUTOLEFT:

    if (ps2x.ButtonPressed(PSB_BLUE))
    {
      resetAutoBooleans();
      STATE = MANUAL;
    }
    autoLeftMode();
    break;



  case AUTORIGHT:

    if (ps2x.ButtonPressed(PSB_BLUE))
    {
      resetAutoBooleans();
      STATE = MANUAL;
    }
    autoRightMode();
    break;



  case AUTOMIDDLE:

    if (ps2x.ButtonPressed(PSB_BLUE))
    {
      resetAutoBooleans();
      STATE = MANUAL;
    }
    autoMiddleMode();

    break;

  default:
    break;

  }
}

/* -------------------------------------------------------------------------- */
/*                                 Controller                                 */
/* -------------------------------------------------------------------------- */

/* ----------------------------- Manual Controls ---------------------------- */

/**
 * This section is all of the manual controls for the robot.
 * By putting all the manual controls into this function it allows you to 
 * easily modify them and if needed to call them in seprate areas.
 * 
 * So this method basically checks the different control inputs in
 * a if statement and then does an action based upon that.   **/
void manualControls()
{

  //Analog Inputs in case I forget 
  //ps2x.Analog(PSS_LY)
  //ps2x.Analog(PSS_RY)

  driveTrainManualDrive(ps2x.Analog(PSS_LY), ps2x.Analog(PSS_RY));

  if (ps2x.Button(PSB_L2))
  {
    gripperOpen();
  }
  else if (ps2x.Button(PSB_R2))
  {
    gripperClose();
  }
  else
  {
    gripperCenterPosition();
  }

  if (ps2x.ButtonPressed(PSB_RED))
  {
    STATE = AUTORIGHT;
  }
  else if (ps2x.ButtonPressed(PSB_PINK))
  {
    STATE = AUTOLEFT;
  }
  else if (ps2x.ButtonPressed(PSB_GREEN))
  {
    STATE = AUTOMIDDLE;
  }

  setServos();
}
/* -------------------------------------------------------------------------- */

/* ------------------------------ Control Setup ----------------------------- */

/** This section of code sets up the controller. It was found online on the 
 * website in the documentation for the project.**/

void controllerSetup()
{
  Serial.begin(115200);

  error = ps2x.config_gamepad(5, 3, 4, 2, true, true); //GamePad(clock, command, attention, data, Pressures?, Rumble?)

  if (error == 0)
  {
    Serial.println("Found Controller, configured successful");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Go to www.billporter.info for updates and to report bugs.");
  }
  else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  type = ps2x.readType();
  switch (type)
  {
  case 0:
    Serial.println("Unknown Controller type");
    break;
  case 1:
    Serial.println("DualShock Controller Found");
    break;
  case 2:
    Serial.println("GuitarHero Controller Found");
    break;
  }
}

/* -------------------------------------------------------------------------- */

/* ----------------------------- Controller Loop ---------------------------- */

/** This section of code was also found online in the project documentation 
 * and is a loop that checks what is currently happening on the controller
 * and reports any errors.  **/
void controllerLoop()
{
  /* You must Read Gamepad to get new values
  Read GamePad and set vibration values
  ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
  if you don't enable the rumble, use ps2x.read_gamepad(); with no values
  you should call this at least once a second
  */
  if (error == 1)
    return;
  if (type == 2)
  {
    ps2x.read_gamepad(); //read controller
    if (ps2x.ButtonPressed(GREEN_FRET))
      Serial.println("Green Fret Pressed");
    if (ps2x.ButtonPressed(RED_FRET))
      Serial.println("Red Fret Pressed");
    if (ps2x.ButtonPressed(YELLOW_FRET))
      Serial.println("Yellow Fret Pressed");
    if (ps2x.ButtonPressed(BLUE_FRET))
      Serial.println("Blue Fret Pressed");
    if (ps2x.ButtonPressed(ORANGE_FRET))
      Serial.println("Orange Fret Pressed");
    if (ps2x.ButtonPressed(STAR_POWER))
      Serial.println("Star Power Command");
    if (ps2x.Button(UP_STRUM)) //will be TRUE as long as button is pressed
      Serial.println("Up Strum");
    if (ps2x.Button(DOWN_STRUM))
      Serial.println("DOWN Strum");
    if (ps2x.Button(PSB_START)) //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if (ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");
    if (ps2x.Button(ORANGE_FRET)) // print stick value IF TRUE
    {
      Serial.print("Wammy Bar Position:");
      Serial.println(ps2x.Analog(WHAMMY_BAR), DEC);
    }
  }
  else
  {                                    //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    if (ps2x.Button(PSB_START))        //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if (ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");
    if (ps2x.Button(PSB_PAD_UP))
    { //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if (ps2x.Button(PSB_PAD_RIGHT))
    {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if (ps2x.Button(PSB_PAD_LEFT))
    {
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if (ps2x.Button(PSB_PAD_DOWN))
    {
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }
    vibrate = ps2x.Analog(PSAB_BLUE); //this will set the large motor vibrate speed based on
                                      //how hard you press the blue (X) button
    if (ps2x.NewButtonState())        //will be TRUE if any button changes state (on to off, or off to on)
    {
      if (ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if (ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if (ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if (ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if (ps2x.Button(PSB_GREEN))
        Serial.println("Triangle pressed");
    }
    if (ps2x.ButtonPressed(PSB_RED)) //will be TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
    if (ps2x.ButtonReleased(PSB_PINK)) //will be TRUE if button was JUST released
      Serial.println("Square just released");
    if (ps2x.NewButtonState(PSB_BLUE)) //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) // print stick values if either is TRUE
    {
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC);
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC);
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC);
    }
  }
  delay(50);
}

/* ---------------------------- Joystick Deadzone --------------------------- */

/** This method takes in a value of the joystick and returns if it is bigger 
 * than a deadzone value. This means the robot won't react to the joystick
 * not being completely accurate. This means the robot will be still 
 * when you are not moving the joysticks even if they are not quite
 * accurate. **/
double joystickDeadzoner(double joystick)
{
  if (joystick > 130)
  {
    return joystick;
  }
  else if (joystick < 125)
  {
    return joystick;
  }
  else
  {
    return 0;
  }
}

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                 Drive Train                                */
/* -------------------------------------------------------------------------- */

/**This section of code contains the all of the code for the Drive Train
 * SubSystem **/

/** This function actually sets the servo positions and is all that needs 
 * to be ran in the loop function. The reasoning behind this was we could
 * add methods later that alter these values durring manual controls. 
 * Allowing for a semi auto mode of sorts with object tracking.
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
  servoGripper.attach(servoGripperPin);
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



/* -------------------------- Manual Drive Function ------------------------- */

/**This will be the main function used to control the drive train. The left
 * and the right parameters will be used with the controllers left and right
 * analog sticks. 
 */

void driveTrainManualDrive(double left, double right)
{

  //Left Logic
  if (left >= 120 && left <= 130)
  {

    servoLeftPositionValue = 1500;
  }

  //This case means it is positive and will use the clockwise map
  else if (left > 128)
  {

    servoLeftPositionValue = (100 * BasicCosineMotionProfile(map(left, 128, 0, 0, 100), 1)) + 1500;
    //servoLeftPositionValue = map(left, 128, 0, 1500, 1600);
  }

  //This case measn it is negetive and will use the counter clockwise map
  else if (left < 128)
  {

    //servoLeftPositionValue = map(left, 255, 128, 1400,1500);
    servoLeftPositionValue = 1500 + (100 * BasicCosineMotionProfile(map(left, 255, 128, 0, 100), 1));
  }

  //Left Logic
  if (right >= 120 && right <= 130)
  {

    servoRightPositionValue = 1500;
  }

  //This case means it is positive and will use the clockwise map
  else if (right > 128)
  {

    //servoRightPositionValue =(map(right, 255, 128,1600,1500));
    servoRightPositionValue = 1600 - (100 * BasicCosineMotionProfile(map(right, 128, 0, 0, 100), 1));
  }

  //This case measn it is negetive and will use the counter clockwise map
  else if (right < 128)
  {

    //servoRightPositionValue = (map(right, 0, 128, 1400, 1500));
    servoRightPositionValue = 1500 - (100 * BasicCosineMotionProfile(map(right, 255, 128, 0, 100), 1));
  }
}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                    Updated Tank Drive Movement Function                    */
/* -------------------------------------------------------------------------- */

/**
 * This function will be a much more simplified version of the other function
 * it will also make way more sense. It will mapped to -100 for full reverse 
 * and full 100 for forward rather than the weird joystick values. Also 
 * we can just map the joystick to -100 to 100. This one also uses a cosine
 * motion profile so it be smooth. 
 *  **/

void tankDriveMovement(double left, double right)
{
  if (left == 0)
  {
    servoLeftPositionValue = 1500;
  }
  else
  {
    servoLeftPositionValue = (100 * BasicCosineMotionProfile(left, 1) + 1500);
  }

  if (right == 0)
  {
    servoRightPositionValue = 1500;
  }
  else
  {
    servoRightPositionValue = (-100 * BasicCosineMotionProfile(right, 1) + 1500);
  }
}

/* -------------------------- Drive Train Commands -------------------------- */
//This method simply stops the drive train.
void driveTrainStop()
{

  double servoLeftPositionValue = 1500;
  double servoRightPositionValue = 1500;
}

/* -------------------------------------------------------------------------- */

/* ---------------------------- Gripper Commands ---------------------------- */
/** This section of code writes different angle values to the gripper
 * servo inturn moving the gripper.
 */
void gripperClose()
{
  servoGripper.write(180);
}

void gripperCenterPosition()
{
  servoGripper.write(90);
}

void gripperOpen()
{
  servoGripper.write(90);
}

/* -------------------------------------------------------------------------- */

/* ----------------------------- Motion Profile ----------------------------- */

                                                                                
/* ---------------------------------- Graph --------------------------------- */

/** This double takes in a x value (input) and returns a y value (output) 
 * according to a cosine curve as seen below in the graph. This is used as 
 * a filter between the analog input from the controller and the power 
 * applied to the motors. So when it is driven correctly it will reduce 
 * jerk on the motors and allow it to be easier to control with slight 
 * adjustments. **/

/**
 * Equation for the graph below 
 * y = 2 * (cos(x + 3.14) + 1)) **/ 

/** The Y axis is the power to the motor and the x axis is the controller input**/

// @&                                                                              
// @&                                                                              
// @&                                                                              
// @&                                                                        .@@
// @&                                                                    @@@       
// @&                                                                .@@.          
// @&                                                              @@(             
// @&                                                            @@                
// @&                                                          @@                  
// @&                                                        @@                    
// @&                                                      %@@                     
// @&                                                     @@                       
// @&                                                   @@(                        
// @&                                                  @@                          
// @&                                                (@@                           
// @&                                               @@                             
// @&                                              @@                              
// @&                                            %@%                               
// @&                                           @@                                 
// @&                                          @@                                  
// @&                                         @@                                   
// @&                                       @@(                                    
// @&                                      @@                                      
// @&                                     @@                                       
// @&                                    @@                                        
// @&                                  @@*                                         
// @&                                 @@                                           
// @&                                @@                                            
// @&                              ,@@                                             
// @&                             @@                                               
// @&                            @@                                                
// @&                           @@                                                 
// @&                         @@                                                   
// @&                        @@                                                    
// @&                      @@#                                                     
// @&                     @@                                                       
// @&                   @@,                                                        
// @&                 *@@                                                          
// @&                @@                                                            
// @&              @@                                                              
// @&           /@@                                                                
// @&         @@/                                                                  
// @&     @@@(                                                                     
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

/* -------------------------------------------------------------------------- */

double BasicCosineMotionProfile(double input, double scale)
{
  if (input < 0)
  {
    /** This section accounts for negative values and returns
     * the value multiplied by -1**/

    return -1 * (2 * (cos(input * scale + 3.14) + 1));
  }
  if (input > 0)
  {
    return 2 * (cos(input * scale + 3.14) + 1);
  }
  else
  {
    return 0;
  }
}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                 Auto Stuff                                 */
/* -------------------------------------------------------------------------- */
/**
 * This section is all of the auto modes and actions
 * 
 **/

/* ------------------------------- Auto Modes ------------------------------- */

/** Auto Modes are sets of auto actions that run in sequence. 
 * For example the left mode closes the gripper, line follows, spins, and then 
 * delivers the object. This are ran in the main state machine.
 * 
 * By making auto modes like this you can easily add different modes and
 * it would make it a lot easier to make fully auto if wanted **/

void autoLeftMode()
{
  closeGripperAutoAction();
  lineFollowUntilCenter();
  spinLeft(900);
  lineFollowAndDeliver(sonarrMaxSpeedCalculation(frontSonarrValue(), 5), 0, .5,.6);
}

void autoRightMode()
{
  closeGripperAutoAction();
  lineFollowUntilCenter();
  spinRight(1200);
  lineFollowAndDeliver(sonarrMaxSpeedCalculation(frontSonarrValue(), 5), 0, .5,.4);
}

void autoMiddleMode()
{
  closeGripperAutoAction();
  lineFollowAndDeliver(sonarrMaxSpeedCalculation(frontSonarrValue(), 5), 0, .5,.4);
}

/* ------------------------------ Auto Actions ------------------------------ */

/** This section contains all of the different auto actions. By making auto 
 * actions we can both abstract directly controlling functions and reuse them
 * as we see fit. **/

/* ------------------------ Basic Auto Action Outline ----------------------- */

/**
 * This is a simple outline of how the auto actions work. I used the move 
 * forward action as an example.
 **/

//Method name and the parmaters of the action. Such as time
void moveForwardTimeBased(double time)
{

  /**This gets the start time of the function by referencing 
   * the system timestamp storing is a temp variable **/
  double starttime = millis();


/** This loop keeps running the action until it is complete 
 * by checking a boolean that is set to true when the action 
 * is complete  **/
  while (autoActionComplete1 == false)
  {
    if (millis() >= time + starttime)
    {
      stop();
      //This is the boolean that is referenced above 
      //At this point the action is complete
      autoActionComplete1 = true;
    }
    else
    {
      //This is where the action will run until complete
      tankMovementNoMotionProfiling(100, 100);
    }
  }
}
 
/* -------------------------------------------------------------------------- */


//Full Forward 1600, 1400
/** This function takes in parameters for moving the left and the right servos
 * based off a range of -100 to 100. -100 is full backwards and 100 is full
 * forwards.  
 */
void tankMovementNoMotionProfiling(double left, double right)
{

  servoLeft.writeMicroseconds(1500 + left);
  servoRight.writeMicroseconds(1500 - right);
}

/* --------------------------- Line Follow Actions -------------------------- */

/** This is the most complex auto action. This action follows a line 
 * and slows down as it gets closer to a wall. It will also deliver an
 * object once it is in a certian range of the wall. 
 * 
 * The reasoning behind the max speed parameter is that to follow 
 * the line we would slow down each one of the wheels a little bit
 * rather than the other way around. This would allow us to mantian 
 * max speed while following the line. This max speed is then adjusted
 * once it is a certian range of the wall as seen below on the graph.
 * 
 * 
 */

void lineFollowAndDeliver(double maxSpeed, double ddelay, double mult,double multBig)
{

  if (maxSpeed < 8)
  {
    tankMovementNoMotionProfiling(0, 0);
    gripperOpen();
    autoActionComplete0 = false;
    autoActionComplete1 = false;
    autoActionComplete2 = false;
    autoActionComplete3 = false;
    STATE = MANUAL;
    
  }

  else if (lineFollowLeftSensor() && !(lineFollowCenterSensor()) && !(lineFollowRightSensor()))
  {

    tankMovementNoMotionProfiling(maxSpeed * multBig, maxSpeed);
    Serial.println("I SHOULD BE TURNING LEFT HARD");
    previousLineFollowState = left;
    delay(ddelay);
  }
  else if (!(lineFollowLeftSensor()) && lineFollowCenterSensor() && !(lineFollowRightSensor()))
  {
    tankMovementNoMotionProfiling(maxSpeed, maxSpeed);
    Serial.println("I SHOULD BE GOING STRAIGHT");
    previousLineFollowState = straight;
    delay(ddelay);
  }
  else if (!(lineFollowLeftSensor()) && !(lineFollowCenterSensor()) && (lineFollowRightSensor()))
  {
    tankMovementNoMotionProfiling(maxSpeed, maxSpeed * multBig);
    Serial.println("I SHOULD BE TURNING RIGHT HARD");
    previousLineFollowState = right;
    delay(ddelay);
  }
  else if ((lineFollowLeftSensor()) && (lineFollowCenterSensor()) && !(lineFollowRightSensor()))
  {

    tankMovementNoMotionProfiling(maxSpeed * mult, maxSpeed);
    Serial.println("I SHOULD BE TURNING LEFT VERY VERY VERY LITTLE");
    previousLineFollowState = left;
    delay(ddelay);
  }
  else if (!(lineFollowLeftSensor()) && (lineFollowCenterSensor()) && (lineFollowRightSensor()))
  {
    tankMovementNoMotionProfiling(maxSpeed, maxSpeed * mult);
    Serial.println("I SHOULD BE TURNING RIGHT VERY VERY VERY LITTLE");
    previousLineFollowState = right;
    delay(ddelay);
  }
  else if ((lineFollowLeftSensor()) && (lineFollowCenterSensor()) && (lineFollowRightSensor()))
  {
    tankMovementNoMotionProfiling(20, 20);
    Serial.println("THIS IS THE ALL WHITE ONE");
    delay(ddelay);
  }
  else
  {

    //ALL BLACK!!!!
    if (!((lineFollowLeftSensor()) && (lineFollowCenterSensor()) && (lineFollowRightSensor())))
    {
      //This breaks if they are not all black!
    }
    //This uses the previousLineFollowState to correct the wrong turn 
    else
    {
      if (previousLineFollowState == left)
      {
        tankMovementNoMotionProfiling(50, 0);
      }
      if (previousLineFollowState == right)
      {
        tankMovementNoMotionProfiling(0, 50);
      }
      else
      {
      }
    }
  }
}

/** This action wil follow the line until the robot reaches the center point 
 * during auto and then it will stop. 
 */

void lineFollowUntilCenter()
{

  while (autoActionComplete1 == false)
  {
    if (!((lineFollowLeftSensor()) && (lineFollowCenterSensor()) && (lineFollowRightSensor())))
    {
      tankMovementNoMotionProfiling(50, 50);
    }
    else
    {
      tankDriveMovement(0, 0);
      autoActionComplete1 = true;
    }
  }
}

/* -------------------------------------------------------------------------- */


//Drivetrain
void driveForward(double power)
{
  servoLeft.writeMicroseconds(1500 + power);
  servoRight.writeMicroseconds(1500 - power);
}

void driveReverse(double power)
{
  servoLeft.writeMicroseconds(1500 - power);
  servoRight.writeMicroseconds(1500 + power);
}

void spinLeft(double time)
{

  double starttime = millis();

  while (autoActionComplete2 == false)
  {
    if (millis() >= time + starttime)
    {
      stop();
      autoActionComplete2 = true;
    }
    else
    {
      servoLeft.writeMicroseconds(1450);
      servoRight.writeMicroseconds(1450);
    }
  }
}



void spinRight(double time)
{
  double starttime = millis();

  while (autoActionComplete2 == false)
  {
    if (millis() >= time + starttime)
    {
      stop();
      autoActionComplete2 = true;
    }
    else
    {
      servoLeft.writeMicroseconds(1550);
      servoRight.writeMicroseconds(1550);
    }
  }
}

void stop()
{
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}

/* ----------------------------- Gripper Actions ---------------------------- */

void closeGripperAutoAction()
{
  while (autoActionComplete0 == false)
  {
    gripperClose();
    delay(10);
    autoActionComplete0 = true;
  }
}

void openGripperAutoAction()
{
  gripperOpen();
  delay(10);
  autoActionComplete0 = false;
  autoActionComplete1 = false;
  autoActionComplete2 = false;
  autoActionComplete3 = false;
  STATE = MANUAL;
}

/* -------------------------------------------------------------------------- */




void resetAutoBooleans()
{
    autoActionComplete0 = false;
    autoActionComplete1 = false;
    autoActionComplete2 = false;
    autoActionComplete3 = false;
}
/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/*                         Line Follow Sensor Booleans                        */
/* -------------------------------------------------------------------------- */

/** These booleans allow us to adjust how sensitive the line following sensors
 * are in this location rather than above in the different auto actions. 
 * 
 * We were going to auto calibrate the light sensors by normalizing the 
 * sensor feedback into a precentage but we found the white value and black 
 * value to be different enough (White was 300 and black was 900) that 
 * it wasn't necessary. 
 */

boolean lineFollowLeftSensor()
{
  if (analogRead(leftLightSensorPin) <= whiteLevel)
  {
    return true;
  }
  else
  {
    return false;
  }
}

boolean lineFollowRightSensor()
{
  if (analogRead(rightLightSensorPin) <= whiteLevel)
  {
    return true;
  }
  else
  {
    return false;
  }
}

boolean lineFollowCenterSensor()
{
  if (analogRead(middleLightSensorPin) <= whiteLevel)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/* -------------------------------------------------------------------------- */
/*                            Sonar Sensor Methods                            */
/* -------------------------------------------------------------------------- */

double frontSonarrValue()
{
  return sonarrValueInches(frontSonarrPin);
}
double leftSonarrValue()
{
  return sonarrValueInches(leftSonarrPin);
}
double rightSonarrValue()
{
  return sonarrValueInches(rightSonarrPin);
}

double sonarrValueInches(int pingPin)
{
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);

  // convert the time into a distance

  return pulseIn(pingPin, HIGH) / 74 / 2;

  delay(100);
}

/** This function will convert the distance away from the wall into a power
 * value for a motor between 0 and 100 given a distance to stop. It does this 
 * expontilly once the distance to stop has been reached. See below for a 
 * example graph of this where the x axis is the distance from the wall 
 * and the y is the motor power
 */

// @@                                                                              
// @@                                                                              
// @@                                                                              
// @@                    ,@@@&&@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@                    @@@                                                       
// @@                   .@@@                                                       
// @@                   @@@.                                                       
// @@                   @@@                                                        
// @@                  (@@(                                                        
// @@                  @@@                                                         
// @@                  @@@                                                         
// @@                 @@@%                                                         
// @@                 @@@                                                          
// @@                .@@@                                                          
// @@                @@@,                                                          
// @@                @@@                                                           
// @@               (@@(                                                           
// @@               @@@                                                            
// @@              ,@@&                                                            
// @@              @@@.                                                            
// @@             .@@@                                                             
// @@             @@@,                                                             
// @@            .@@@                                                              
// @@            @@@,                                                              
// @@           .@@@                                                               
// @@           @@@.                                                               
// @@          ,@@@                                                                
// @@          @@@                                                                 
// @@         (@@%                                                                 
// @@         @@@                                                                  
// @@        @@@,                                                                  
// @@       #@@@                                                                   
// @@       @@@                                                                    
// @@      @@@                                                                     
// @@     @@@,                                                                     
// @@    @@@(                                                                      
// @@   @@@(                                                                       
// @@  @@@.                                                                        
// @@@@@@                                                                          
// @@@@&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


double sonarrMaxSpeedCalculation(double distanceAway, double distanceToStop)
{

  if (((1 / (distanceToStop * distanceToStop)) * 100 * distanceAway * distanceAway) < 100)
  {
    return ((1 / (distanceToStop * distanceToStop)) * 100 * (distanceAway) * (distanceAway));
  }
  else
  {
    return 100;
  }
}

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                               Sensor Values!                               */
/* -------------------------------------------------------------------------- */

/** This section of code prints off the various sensor values on the robot. 
 * We used this method to debug code.
 */
void printSensorValues()
{
  
  Serial.print("| Middle Sensor: ");
  Serial.print(analogRead(middleLightSensorPin));
  Serial.println("");
  Serial.print("-------------------------");
  Serial.println("");
  Serial.print(lineFollowCenterSensor());
  Serial.print("| Right Sensor: ");
  Serial.print(analogRead(rightLightSensorPin));
  Serial.println("");
  Serial.print("---------------------------");
  Serial.println("");
  Serial.print(lineFollowRightSensor());
  Serial.print("| Left Sensor: ");
  Serial.print(analogRead(leftLightSensorPin));
  Serial.println("");
  Serial.print("--------------------");
  Serial.print(lineFollowLeftSensor());
  Serial.println("");

  Serial.println("");
  Serial.println("--------------------------------------");
  Serial.println("");

  Serial.print("| Front Sonarr: ");
  Serial.print(frontSonarrValue());
  Serial.print("| Right Sonarr: ");
  Serial.print(rightSonarrValue());
  Serial.print("| Left Sonarr: ");
  Serial.print(leftSonarrValue());

  Serial.println("");
  Serial.println("--------------------------------------");
  Serial.println("");
}
 

