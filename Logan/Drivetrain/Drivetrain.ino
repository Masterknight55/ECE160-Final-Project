
#include <PS2X_lib.h>   
PS2X ps2x; 
//right now, the library does NOT support hot-pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.
int error = 0; 
byte type = 0;
byte vibrate = 0;



#include <Servo.h>
Servo servoLeft;                                   
Servo servoRight;
Servo servoGripper;  



/* --------------------------------- Pinout --------------------------------- */

const int servoLeftPin = 13;                       
const int servoRightPin = 10;
const int servoGripperPin = 11;

/** This is the global value for the servo code
 * by setting this global varible in our code 
 * for the servo power we won't deal with as much conflicts
 * with setting the servo. 
 */
double servoLeftPositionValue = 1500;
double servoRightPositionValue = 1500; 

/* -------------------------------------------------------------------------- */


void setup() {

controllerSetup();
  // put your setup code here, to run once:
  setupServos();
}


void loop()
{
  // put your main code here, to run repeatedly:
  controllerLoop();
  manualControls();
  setServos();
  //driveTrainManualDrive(100,100);
  

}


/* -------------------------------------------------------------------------- */
/*                                 Controller                                 */
/* -------------------------------------------------------------------------- */

void manualControls()
{
  
  driveTrainManualDrive(ps2x.Analog(PSS_LY),ps2x.Analog(PSS_RY));



}




void controllerSetup()
{
  Serial.begin(9600);

error = ps2x.config_gamepad(5,3,4,2, true, true);   //GamePad(clock, command, attention, data, Pressures?, Rumble?) 

if(error == 0){
  Serial.println("Found Controller, configured successful");
  Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
 Serial.println("holding L1 or R1 will print out the analog stick values.");
 Serial.println("Go to www.billporter.info for updates and to report bugs.");
}
 else if(error == 1)
  Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
 else if(error == 2)
  Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
 else if(error == 3)
  Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  type = ps2x.readType(); 
    switch(type) {
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

void controllerLoop()
{
  /* You must Read Gamepad to get new values
  Read GamePad and set vibration values
  ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
  if you don't enable the rumble, use ps2x.read_gamepad(); with no values
  you should call this at least once a second
  */
if(error == 1) 
 return; 
if(type == 2){ 
  ps2x.read_gamepad();          //read controller 
  if(ps2x.ButtonPressed(GREEN_FRET))
    Serial.println("Green Fret Pressed");
  if(ps2x.ButtonPressed(RED_FRET))
    Serial.println("Red Fret Pressed");
  if(ps2x.ButtonPressed(YELLOW_FRET))
    Serial.println("Yellow Fret Pressed");
  if(ps2x.ButtonPressed(BLUE_FRET))
    Serial.println("Blue Fret Pressed");
  if(ps2x.ButtonPressed(ORANGE_FRET))
    Serial.println("Orange Fret Pressed");
   if(ps2x.ButtonPressed(STAR_POWER))
    Serial.println("Star Power Command");
   if(ps2x.Button(UP_STRUM))          //will be TRUE as long as button is pressed
    Serial.println("Up Strum");
   if(ps2x.Button(DOWN_STRUM))
    Serial.println("DOWN Strum");
   if(ps2x.Button(PSB_START))                   //will be TRUE as long as button is pressed
        Serial.println("Start is being held");
   if(ps2x.Button(PSB_SELECT))
        Serial.println("Select is being held");
   if(ps2x.Button(ORANGE_FRET)) // print stick value IF TRUE
   {
       Serial.print("Wammy Bar Position:");
       Serial.println(ps2x.Analog(WHAMMY_BAR), DEC); 
   } 
}
else { //DualShock Controller
   ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed
   if(ps2x.Button(PSB_START))                   //will be TRUE as long as button is pressed
        Serial.println("Start is being held");
   if(ps2x.Button(PSB_SELECT))
        Serial.println("Select is being held");
    if(ps2x.Button(PSB_PAD_UP)) {         //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
     }
     if(ps2x.Button(PSB_PAD_RIGHT)){
      Serial.print("Right held this hard: ");
       Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
     }
     if(ps2x.Button(PSB_PAD_LEFT)){
      Serial.print("LEFT held this hard: ");
       Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
     }
     if(ps2x.Button(PSB_PAD_DOWN)){
      Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
     }   
     vibrate = ps2x.Analog(PSAB_BLUE);        //this will set the large motor vibrate speed based on 
                                             //how hard you press the blue (X) button    
   if (ps2x.NewButtonState())               //will be TRUE if any button changes state (on to off, or off to on)
   {   
       if(ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
       if(ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
       if(ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
       if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
       if(ps2x.Button(PSB_GREEN))
        Serial.println("Triangle pressed");
   }   
   if(ps2x.ButtonPressed(PSB_RED))             //will be TRUE if button was JUST pressed
        Serial.println("Circle just pressed");
   if(ps2x.ButtonReleased(PSB_PINK))             //will be TRUE if button was JUST released
        Serial.println("Square just released");     
   if(ps2x.NewButtonState(PSB_BLUE))            //will be TRUE if button was JUST pressed OR released
        Serial.println("X just changed");    
   if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) // print stick values if either is TRUE
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


/**This will be the main function used to control the drive train. The left
 * and the right parameters will be used with the controllers left and right
 * analog sticks. 
 */


/* -------------------------- Manual Drive Function ------------------------- */

//TODO Figure out the values for the analog stick. Righ now they are at -100 and 100 
void driveTrainManualDrive(double left, double right)
{

//Left Logic
if(left>=120 && left <= 130)
{

servoLeftPositionValue = 1500;

}


//This case means it is positive and will use the clockwise map  
else if(left > 128)
{

servoLeftPositionValue = map(left, 128, 0, 1500, 1700);

}

//This case measn it is negetive and will use the counter clockwise map
else if(left < 128)
{

servoLeftPositionValue = map(left, 255, 128, 1300,1500 );

}

//Left Logic
if(right>=120 && right <= 130)
{

servoRightPositionValue = 1500;

}


//This case means it is positive and will use the clockwise map  
else if(right > 128)
{

servoRightPositionValue =-(map(right, 255, 128,1700,1500));



}

//This case measn it is negetive and will use the counter clockwise map
else if(right < 128)
{

servoRightPositionValue = -(map(right, 0, 128, 1300, 1500));

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
servoGripper.write(0);
}


void gripperCenterPosition()
{
servoGripper.write(90);
}


void gripperOpen()
{
servoGripper.write(180);
}

/* -------------------------------------------------------------------------- */


