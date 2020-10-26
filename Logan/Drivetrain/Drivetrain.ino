
/* --------------------------------- Imports -------------------------------- */

//#include <SPI.h>
//#include <NRFLite.h>


//#include <nRF24L01.h>
//#include <NRFLite.h>


#include <PS2X_lib.h>


PS2X ps2x; 

int error = 0; 
byte type = 0;
byte vibrate = 0;



#include <Servo.h>
Servo servoLeft;                                   
Servo servoRight;
Servo servoGripper;  

//int IRPin = 12;
//IRrecv myIR(IRPin);
//decode_results results;

//Reciver Pins and stuff
//NRFLite _radio;
//unsigned long _data;

//int x = 0;
//int y = 0;
//int button = 0;


/* --------------------------------- Pinout --------------------------------- */

const int servoLeftPin = 13;                       
const int servoRightPin = 12;
const int servoGripperPin = 10;

int rightLightSensorPin = 2; // Left Sensor on Analog Pin 2
int leftLightSensorPin = 1; // Right Sensor on Analog Pin 1
int middleLightSensorPin = 0; // Middle Sensor on Analog Pin 0

double whiteLevel = 0;
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

void setup() {



  // put your setup code here, to run once:
  setupServos();
  
  controllerSetup();
  //Trans 
  //reciverSetup();
}


void loop()
{
  // put your main code here, to run repeatedly:
  controllerLoop();
  manualControls();

  //Trans 
  //reciverLoop();
  
  setServos();
  

}


/* -------------------------------------------------------------------------- */
/*                                 Controller                                 */
/* -------------------------------------------------------------------------- */


/* ----------------------------- Manual Controls ---------------------------- */

void manualControls()
{

  //driveTrainManualDrive(ps2x.Analog(PSS_LY), ps2x.Analog(PSS_RY)); 
  //ps2x.Analog(PSS_LY)
  //ps2x.Analog(PSS_RY)

  driveTrainManualDrive(ps2x.Analog(PSS_LY), ps2x.Analog(PSS_RY));
  
//tankDriveMovement(map(joystickDeadzoner(ps2x.Analog(PSS_LY)), 0,255,-100,100), map(joystickDeadzoner(ps2x.Analog(PSS_RY)), 0,255,-100,100));
  
if(ps2x.Button(PSB_L2))
{
  gripperOpen();
}
else if(ps2x.Button(PSB_R2))
{
  gripperClose();
}
else
{
  gripperCenterPosition();
}

}
/* -------------------------------------------------------------------------- */


/* ------------------------------ Control Setup ----------------------------- */

void controllerSetup()
{
  Serial.begin(115200);

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

/* -------------------------------------------------------------------------- */


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


double joystickDeadzoner(double joystick)
{
  if(joystick > 130)
  {
    return joystick;
  }
  else if(joystick < 125)
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
 * 
 * 
 */


/* -------------------------- Manual Drive Function ------------------------- */

 
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


servoLeftPositionValue = ( 100 * BasicCosineMotionProfile(map(left,128,0,0,100) ,1)) + 1500; 
//servoLeftPositionValue = map(left, 128, 0, 1500, 1600);

}

//This case measn it is negetive and will use the counter clockwise map
else if(left < 128)
{

//servoLeftPositionValue = map(left, 255, 128, 1400,1500);
servoLeftPositionValue = 1500 + (100 * BasicCosineMotionProfile(map(left,255,128,0,100) ,1));
}

//Left Logic
if(right>=120 && right <= 130)
{

servoRightPositionValue = 1500;

}


//This case means it is positive and will use the clockwise map  
else if(right > 128)
{

//servoRightPositionValue =(map(right, 255, 128,1600,1500));
servoRightPositionValue = 1600 - ( 100 * BasicCosineMotionProfile(map(right,128,0,0,100) ,1));


}

//This case measn it is negetive and will use the counter clockwise map
else if(right < 128)
{

//servoRightPositionValue = (map(right, 0, 128, 1400, 1500));
servoRightPositionValue = 1500 - (100 * BasicCosineMotionProfile(map(right,255,128,0,100) ,1));
}


}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                    Updated Tank Drive Movement Function                    */
/* -------------------------------------------------------------------------- */

/**
 * This function will be a much more simplfied version of the other function
 * it will also make way more sense. It will mapped to -100 for full reverse 
 * and full 100 for forward rather than the weird joystick values. Also 
 * we can just map the joystick to -100 to 100. This one also uses a cosine
 * motion profile so it be smoooooooooth. 
 *  **/

void tankDriveMovement(double left, double right)
{
if(left == 0)
{
servoLeftPositionValue = 1500;  
}
else
{
servoLeftPositionValue =  (100 * BasicCosineMotionProfile(left,1) + 1500);
}

if(right == 0)
{
  servoRightPositionValue = 1500;
}
else
{
  servoRightPositionValue = (-100 * BasicCosineMotionProfile(right,1) + 1500);
}

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

double servoLeftPositionValue = 1400;
double servoRightPositionValue = 1600;

}

//Conuter clockwise is reverse
void driveTrainReverse()
{

double servoLeftPositionValue = 1700;
double servoRightPositionValue = 1300;

}

void driveTrainSpinLeft()
{

double servoLeftPositionValue = 1700;
double servoRightPositionValue = 1700;

}

void driveTrainSpinRight()
{

double servoLeftPositionValue = 1300;
double servoRightPositionValue = 1300;

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
  double BasicCosineMotionProfile(double input, double scale)
    {
        if(input < 0)
			{
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
 * This sectino of code is for all of the auto stuff. There are modes for 
 * the different delivery options and there are actions for the different 
 * actions the robot can do.  
 * **/




/* ------------------------------- Auto Modes ------------------------------- */






/* ------------------------------ Auto Actions ------------------------------ */



//Gripper Commands
void closeGripper()
{

}

void openGripper()
{

}


//Line Following
void lineFollow()
{

}

void lineFollowWithSonarSlowdown()
{

}

//Drivetrain 
void driveForward(double power)
{
servoLeft.writeMicroseconds(1500+power);
servoRight.writeMicroseconds(1500-power);
}

void driveReverse(double power)
{
servoLeft.writeMicroseconds(1500-power);
servoRight.writeMicroseconds(1500+power);
}



void spinLeft()
{
servoLeft.writeMicroseconds(1600);
servoRight.writeMicroseconds(1600);
}

void spinRight()
{
servoLeft.writeMicroseconds(1400);
servoRight.writeMicroseconds(1400);
}



//Booleans 
boolean isInCenter()
{
  return false;
}

boolean wallToLeft()
{
  return false;
}

boolean walltoRight()
{
  return false;
}


//Line Following Booleans
void lineFollowCalibrate()
{

}


boolean lineFollowLeftSensor()
{
  if(readQD(leftLightSensorPin) <= whiteLevel)
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
  if(readQD(rightLightSensorPin) <= whiteLevel)
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
  if(readQD(middleLightSensorPin) <= whiteLevel)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//Sonarr Sensors
double frontSonarrValue()
{
  return 0.0;
}
double leftSonarrValue()
{
  return 0.0;
}
double rightSonarrValue()
{
  return 0.0;
}





























// /* -------------------------------------------------------------------------- */
// /*                            Transmitter Functions                           */
// /* -------------------------------------------------------------------------- */

// void reciverSetup()
// {
//   Serial.begin(115200);
// _radio.init(3, 9, 10); // Set this radio's Id = 1, along with its CE and CSN pins
// }

// void reciverLoop()
// {
// while (_radio.hasData())
//     {
//         _radio.readData(&_data);

//         x = (_data & 2095104)>>11;

//         y = (_data & 2046)>>1;

//         button = _data & 1;

        
//         Serial.println(_data,BIN);
//         //Serial.println("X Axis:");
//         Serial.println(x);
//         //Serial.println("Y Axis:");
//         Serial.println(y);
//         Serial.println(button);
  
//   if(button == 1)
//   {
//     //180 closed 
//     servoGripper.write(180);
    

//   }
//   else
//   {
//     //90
//     servoGripper.write(90);

    
//   }
  

//   if(x > 40)
//   {
//     servoLeft.writeMicroseconds(1700);
//     servoRight.writeMicroseconds(1300);
//   }
//   else if((x < 40) && (x > 5))
//   {
//     servoLeft.writeMicroseconds(1300);
//     servoRight.writeMicroseconds(1700);
//   }
//   else if(y > 530)
//   {
//     servoLeft.writeMicroseconds(1700);
//     servoRight.writeMicroseconds(1700);
//   }
//   else if(y < 520)
//   {
//     servoLeft.writeMicroseconds(1300);
//     servoRight.writeMicroseconds(1300);
//   }
//   else
//   {
//     servoLeft.writeMicroseconds(1500);
//     servoRight.writeMicroseconds(1500);
//   }
        
//     }

<<<<<<< Updated upstream
// }
=======
}



void SensorRead()
{

  Serial.println()


}
>>>>>>> Stashed changes
