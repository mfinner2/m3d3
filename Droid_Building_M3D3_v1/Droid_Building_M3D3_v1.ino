// =======================================================================================
//                     PS3 Starting Sketch for Notre Dame Droid Class
// =======================================================================================
//                          Last Revised Date: 01/08/2023
//                             Revised By: Prof McLaughlin
// =======================================================================================
// ---------------------------------------------------------------------------------------
//                          Libraries
// ---------------------------------------------------------------------------------------
#include <PS3BT.h>
#include <usbhub.h>
#include <Sabertooth.h>
#include <Adafruit_TLC5947.h>
#include <MP3Trigger.h>
#include <Servo.h> 
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NewPing.h>

// ---------------------------------------------------------------------------------------
//    Constants
// ---------------------------------------------------------------------------------------
#define SABERTOOTH_ADDR 128

// ---------------------------------------------------------------------------------------
//                 Setup for USB, Bluetooth Dongle, & PS3 Controller
// ---------------------------------------------------------------------------------------
USB Usb; //declaring USB port
BTD Btd(&Usb); //indicating Bluetooth device at the USB port
PS3BT *PS3Controller=new PS3BT(&Btd); //identifying bluetooth device as PS3 controller

// ---------------------------------------------------------------------------------------
//    Used for PS3 Fault Detection
// ---------------------------------------------------------------------------------------
uint32_t msgLagTime = 0;
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;
byte joystickDeadZoneRange = 15;

boolean isPS3ControllerInitialized = false;
boolean mainControllerConnected = false;
boolean WaitingforReconnect = false;
boolean isFootMotorStopped = true;

// ---------------------------------------------------------------------------------------
//    Used for PS3 Controller Request Management
// ---------------------------------------------------------------------------------------
long previousRequestMillis = millis();
boolean extraRequestInputs = false;

// ---------------------------------------------------------------------------------------
//    Request State Machine Variables for PS3 Controller
// ---------------------------------------------------------------------------------------

// Main state varable to determine if a request has been made by the PS3 Controller
boolean reqMade = false;
boolean reqLeftJoyMade = false;
boolean reqRightJoyMade = false;

// LEFT & RIGHT Joystick State Request Values
boolean reqLeftJoyUp = false;
boolean reqLeftJoyDown = false;
int reqLeftJoyYValue = 0;

boolean reqLeftJoyLeft = false;
boolean reqLeftJoyRight = false;
int reqLeftJoyXValue = 0;

boolean reqRightJoyUp = false;
boolean reqRightJoyDown = false;
int reqRightJoyYValue = 0;

boolean reqRightJoyLeft = false;
boolean reqRightJoyRight = false;
int reqRightJoyXValue = 0;

// PS3 Controller Button State Variables
boolean reqArrowUp = false;
boolean reqArrowDown = false;
boolean reqArrowLeft = false;
boolean reqArrowRight = false;
boolean reqCircle = false;
boolean reqCross = false;
boolean reqTriangle = false;
boolean reqSquare = false;
boolean reqL1 = false;
boolean reqL2 = false;
boolean reqR1 = false;
boolean reqR2 = false;
boolean reqSelect = false;
boolean reqStart = false;
boolean reqPS = false;

// ---------------------------------------------------------------------------------------
//    Used for Pin 13 Main Loop Blinker
// ---------------------------------------------------------------------------------------
long blinkMillis = millis();
boolean blinkOn = false;

// ---------------------------------------------------------------------------------------
//    Used for Servo
// ---------------------------------------------------------------------------------------
Servo myServo;
long servoTimer = millis();
boolean servoMoving = false;
int servoPosition = 90;

// ---------------------------------------------------------------------------------------
//    Used for Motor Controller
// ---------------------------------------------------------------------------------------
int driveDeadBandRange = 10;
Sabertooth *ST = new Sabertooth(SABERTOOTH_ADDR, Serial1); //tx 1 (pin 18)

int currentSpeed = 0;
int currentTurn = 0;
boolean droidMoving = false;

// ---------------------------------------------------------------------------------------
//    Used for Sonars
// ---------------------------------------------------------------------------------------
NewPing frontSonar = NewPing(40, 41); //trig on 34, echo on 35
NewPing leftFrontSonar = NewPing(38, 39); 
NewPing backSonar = NewPing(34, 35); 
NewPing leftBackSonar = NewPing(36, 37);

 boolean autoMode = false;
 int sonarReadCycle = 0; //can't ping every sonar every cycle, so only calculate one per cycle, identify which sonar with this?
 long sonarIntervalTimer = millis();
 int sonarIntervalTime = 300;
 int currentFrontDistance = -1; //distance in centimeters
 int currentLeftFrontDistance = -1;
 int currentBackDistance = -1;
 int currentLeftBackDistance = -1;
 int tapeDistanceFront = -1; //distance from tape to wall, will need to detect
 int tapeDistanceBack = -1; //distance from tape to wall, will need to detect
 int autonCounter = 0;
 boolean autonSetFinished = false;
 int autonSwerveVal = 0;
 long turnTimer = millis();

 // ---------------------------------------------------------------------------------------
//    Used for Sound
// ---------------------------------------------------------------------------------------
 MP3Trigger myMP3Trigger;

 typedef struct {
  char soundName[20];
  long millisecs;
 }Sound;

 Sound soundsArray[8];
 long soundTimer = millis();
 int soundInterval = -1;
 int track = 0;

 bool changeTrack = false;

 bool ambientPlaying = true;


// ---------------------------------------------------------------------------------------
//    Used for OLED
// ---------------------------------------------------------------------------------------

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------------------------------------------------------------------------------------
//    Used for LEDs
// ---------------------------------------------------------------------------------------
#define CLOCK 5
#define DATA 4
#define LATCH 6
Adafruit_TLC5947 LEDControl = Adafruit_TLC5947(1, CLOCK, DATA, LATCH);
int ledMaxBright = 4000; //technically 4095, can go down to 0
const int numLEDs = 6;

int ambient1[numLEDs];
int ambient2[numLEDs];
int ambient3[numLEDs];
int ambient4[numLEDs];
int ambient5[numLEDs];


// =======================================================================================
//                                 Main Program
// =======================================================================================
// =======================================================================================
//                                Setup Function
// =======================================================================================
void setup()
{
  
    //Initialize Serial @ 115200 baud rate for Serial Monitor Debugging
    Serial.begin(115200);
    while (!Serial); //can only write in setup, not in loop
    
    //Initialize the USB Dongle and check for errors
    if (Usb.Init() == -1)
    {
        Serial.println("OSC did not start");
        while (1); //halt
    }
    
    Serial.println("Bluetooth Library Started");
    
    //PS3 Controller - sets the interrupt function to call when PS3 controller tries to connect
    PS3Controller->attachOnInit(onInitPS3Controller); 

    //Setup PIN 13 for Arduino Main Loop Blinker Routine
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

   // ----------------------------------------------
   // YOUR SETUP CONTROL CODE SHOULD START HERE
   // ----------------------------------------------

   //Servo
   myServo.attach(9);
   myServo.write(servoPosition);

   //Sound
   myMP3Trigger.setup(&Serial2);
   Serial2.begin(MP3Trigger::serialRate());
   randomSeed(analogRead(0)); //note: change if anything gets connected to pin 0
   soundInterval = random(1000, 5000);

   strcpy(soundsArray[0].soundName, " ");
   soundsArray[0].millisecs = 0;
   strcpy(soundsArray[1].soundName, "Rattling Cans");
   soundsArray[1].millisecs = 1500;
   strcpy(soundsArray[2].soundName, "Shifting Cans");
   soundsArray[2].millisecs = 1300;
   strcpy(soundsArray[3].soundName, "Throw Pastic");
   soundsArray[3].millisecs = 1200;
   strcpy(soundsArray[4].soundName, "Crumple");
   soundsArray[4].millisecs = 1000;
   strcpy(soundsArray[5].soundName, "Glass Bottle");
   soundsArray[5].millisecs = 1000;
   strcpy(soundsArray[6].soundName, "Rolling");
   soundsArray[6].millisecs = 680;
   strcpy(soundsArray[7].soundName, "Bin Close");
   soundsArray[7].millisecs = 1000;

   //Motor Control
   Serial1.begin(9600);
   ST->autobaud();
   ST->setTimeout(200); //if contact is lost, stop (must speak to 5 times a second)
   ST->setDeadband(driveDeadBandRange);

   // OLED
   display.begin(SSD1306_SWITCHCAPVCC, 0X3C);
   display.display();
   delay(2000);
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.clearDisplay();
   display.display();

   //LEDs
   LEDControl.begin();
   for (int i = 0; i < numLEDs; i++) {
    LEDControl.setPWM(i, 0);
   }
   LEDControl.write();

    for (int i = 0; i < numLEDs; i++) {
      ambient1[i] = (i % 2 == 0) ? ledMaxBright : 0;
      ambient2[i] = (i % 3 == 1) ? ledMaxBright : ((i % 3 == 2) ? ledMaxBright/2 : 0);
      ambient3[i] = (i % 3 == 2) ? ledMaxBright : 0;
      ambient4[i] = (i % 3 == 0) ? ledMaxBright : ((i % 2 == 0) ? ledMaxBright/3 : 0);
      ambient5[i] = (i % 2 == 1) ? ledMaxBright : 0;
    }
   

   // ----------------------------------------------
   // YOUR SETUP CONTROL CODE SHOULD END HERE
   // ---------------------------------------------
}

// =======================================================================================
//    Main Program Loop - This is the recurring check loop for entire sketch
// =======================================================================================
void loop()
{   
   // Make sure the PS3 Controller is working - skip main loop if not
   if ( !readUSB() )
   {
     return;
   }

   // If the PS3 controller has been connected - start processing the main controller routines
   if (PS3Controller->PS3Connected) {
   
       // Read the PS3 Controller and set request state variables for this loop
       readPS3Request();
    
       // ----------------------------------------------
       // YOUR MAIN LOOP CONTROL CODE SHOULD START HERE
       // ----------------------------------------------
    
       // Sample droid function call from PS3 request - REMOVE ONCE YOU UNDERSTAND STRUCTURE
       if (reqCross) {
          if (autoMode == true) {
            autoMode = false;
          } else {
            autoMode = true;
            sonarIntervalTimer = millis();
            currentFrontDistance = -1; //distance in centimeters
            currentLeftFrontDistance = -1;
            currentBackDistance = -1;
            currentLeftBackDistance = -1;
            tapeDistanceFront = -1;
            tapeDistanceBack = -1;
            autonCounter = 0;
            autonSetFinished = false;
          }
       }
       if (autoMode == false) {
       if (reqArrowUp) {
          servoUp();
       }

       if (reqArrowDown) {
          servoDown();
       }

       moveDroid();
       }

       if (autoMode == true) {
        takeSonarReadings();
        Serial.print("Front: ");
          Serial.println(currentFrontDistance);
          Serial.print("Left Front: ");
          Serial.println(currentLeftFrontDistance);
          Serial.print("Left Back: ");
          Serial.println(currentLeftBackDistance);
          Serial.print("Back: ");
          Serial.println(currentBackDistance);
        if (!(tapeDistanceFront == -1 || tapeDistanceBack == -1)) {
        if (autonCounter == 0 || autonCounter == 2 || autonCounter == 4 || autonCounter == 6) {
           moveForward();
        } else if (autonCounter == 1) {
          turnRight();
        }
        if (autonSetFinished) {
          stopDrive();
          autonCounter++;
          turnTimer = millis();
          autonSetFinished = false;
        }
        /*
          Serial.print("Front: ");
          Serial.println(currentFrontDistance);
          Serial.print("Left Front: ");
          Serial.println(currentLeftFrontDistance);
          Serial.print("Left Back: ");
          Serial.println(currentLeftBackDistance);
          Serial.print("Back: ");
          Serial.println(currentBackDistance);
          if (autonSetFinished) {
            stopDrive();
            autonCounter++;
            turnTimer = millis();
          }
          if (autonCounter == 0 || autonCounter == 2 || autonCounter == 4 || autonCounter == 6) {
            moveForward();
          }
          if (autonCounter == 1 || autonCounter == 3 || autonCounter == 5) {
            turnRight();
          }
          if (autonCounter == 7 || autonCounter == 9 || autonCounter == 11 || autonCounter == 13) {
            moveBackward();
          }
          if (autonCounter == 8 || autonCounter == 10 || autonCounter == 12) {
            turnRightBackward();
          } */
        } else {
          tapeDistanceFront = currentLeftFrontDistance;
          tapeDistanceBack = currentLeftBackDistance;
        }
        //front should be about 2 less than dist from left before turn
       }

       

       if (droidMoving && ambientPlaying) {
          ambientPlaying = false;
          soundInterval = 0;
          movementSound(); 
          changeTrack = true;
        }

        if (droidMoving) {
          movementSound();
        }

        if (!droidMoving && !ambientPlaying) {
          ambientPlaying = true;
          soundInterval = 0;
        }

        if (ambientPlaying) {
          ambientNoise();
        }

        if (changeTrack) {
          displaySound();
        }

       myMP3Trigger.update();
        
       // ----------------------------------------------
       // YOUR MAIN LOOP CONTROL CODE SHOULD END HERE
       // ----------------------------------------------
      
       // Ignore extra inputs from the PS3 Controller for 1/2 second from prior input
       if (extraRequestInputs)
       {
          if ((previousRequestMillis + 500) < millis())
          {
              extraRequestInputs = false;
          }
       }
    
       // If there was a PS3 request this loop - reset the request variables for next loop
       if (reqMade) {
           resetRequestVariables();
           reqMade = false;
       } 
   }

   // Blink to show working heart beat on the Arduino control board
   // If Arduino LED is not blinking - the sketch has crashed
   if ((blinkMillis + 500) < millis()) {
      if (blinkOn) {
        digitalWrite(13, LOW);
        blinkOn = false;
      } else {
        digitalWrite(13, HIGH);
        blinkOn = true;
      }
      blinkMillis = millis();
   }
}

// =======================================================================================
//      ADD YOUR CUSTOM DROID FUNCTIONS STARTING HERE
// =======================================================================================

// SAMPLE CUSTOM DROID FUNCTION from PS3 REQUEST - REMOVE ONCE YOU UNDERSTAND STRUCTURE
long M3D3Max(long a, long b) {
    if (a > b) {
      return a;
    }
    return b;
}

long M3D3Min(long a, long b) {
    if (a < b) {
      return a;
    }
    return b;
}

long M3D3Abs(long a) {
  if (a < 0) {
    return -1 * a;
  }
  return a;
}

int sign(int a) {
  if (a < 0) {
     return -1;
  }
  return 1;
}
void servoUp()
{
    //Serial.println("Droid is now executing my custom ARROW UP function");
    myServo.write(5);
    servoPosition = 5;
}

void servoDown()
{
    //Serial.println("Droid is now executing my custom ARROW DOWN function");
    myServo.write(90);
    servoPosition = 90;
}

void moveServoByJoystick() {
  if (reqLeftJoyLeft && servoPosition >= 7 && ((servoTimer + 1000) < millis())) {
    servoPosition = servoPosition - 2;
    myServo.write(servoPosition);
    servoTimer = millis();
    Serial.println(servoPosition);
  }
  if (reqLeftJoyRight && servoPosition <= 168 && ((servoTimer + 1000) < millis())) {
    servoPosition = servoPosition + 2;
    myServo.write(servoPosition);
    servoTimer = millis();
    Serial.println(servoPosition);
  }
}

void moveDroid() { //not finished, but a start
  if (reqLeftJoyMade || reqRightJoyMade) {
    currentSpeed = M3D3Max(M3D3Min(currentSpeed + 1, reqLeftJoyYValue * 2 / 3), currentSpeed - 1);
    currentTurn = M3D3Min(160 - M3D3Abs(currentSpeed), M3D3Max(-1 * (160 - M3D3Abs(currentSpeed)), reqRightJoyXValue/3));
    ST->turn(currentTurn);
    ST->drive(currentSpeed);
    if (!droidMoving) {
      droidMoving = true;
    }
  } else {
    if (droidMoving) {
      currentSpeed = M3D3Min(M3D3Max(currentSpeed - 1, 0), currentSpeed + 1);
      currentTurn = 0;
      if (currentSpeed) {
        ST->turn(currentTurn);
        ST->drive(currentSpeed);
      } else {
      ST->stop();
      droidMoving = false;
      currentTurn = 0;
      currentSpeed = 0;
      }
    }
  }
}

void takeSonarReadings() {
  if ((sonarIntervalTimer + sonarIntervalTime) < millis()) {
    return;
} else {
  sonarIntervalTimer = millis();
}

if (sonarReadCycle == 0) {
  currentFrontDistance = frontSonar.convert_cm(frontSonar.ping_median(5));
} else if (sonarReadCycle == 1) {
  currentLeftFrontDistance = leftFrontSonar.convert_cm(leftFrontSonar.ping_median(5));
} else if (sonarReadCycle == 2) {
  currentLeftBackDistance = leftBackSonar.convert_cm(leftBackSonar.ping_median(5));
} else if (sonarReadCycle == 3) {
  currentBackDistance = backSonar.convert_cm(backSonar.ping_median(5));
}
sonarReadCycle = (sonarReadCycle + 1) % 4;
}

void moveForward() {
  if (currentFrontDistance - ((tapeDistanceFront + tapeDistanceBack)/2) > 3) {
    autonSwerveVal = 0;
    if (tapeDistanceFront < currentLeftFrontDistance) {
      autonSwerveVal -= 4;
    }
    if (tapeDistanceFront > currentLeftFrontDistance) {
      autonSwerveVal += 4;
    }
    ST->turn(autonSwerveVal);
    ST->drive(-50);
  } else {
    autonSetFinished = true;
  }
  
}

void turnRight() {
  if ((turnTimer + 750) > millis()) { //bumped up since it might be catching the wall at the last second
    ST->turn(60);
    ST->drive(0);
  } else {
    autonSetFinished = true;
  }
  
}

void moveBackward() {
  
}

void turnRightBackward() {
  
}

void stopDrive() {
  ST->stop();
}

void ambientNoise() {
  
  if (track != 0 && soundTimer + soundsArray[track].millisecs < millis()) {
    changeTrack = true;
    track = 0;
  }
  
  if (soundTimer + soundInterval < millis()) {
    //randomize track
    changeTrack = true;
    track = random(1, 6);
    myMP3Trigger.setVolume(50);
    myMP3Trigger.trigger(track /*+ offset*/);
    soundTimer = millis();
    soundInterval = random(1000, 5000) + soundsArray[track].millisecs;
    ambientLights();
    
  }
}

void ambientLights() {
  if (track == 1) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, ambient1[i]);
    }
  } else if (track == 2) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, ambient2[i]);
    }
  } else if (track == 3) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, ambient3[i]);
    }
  } else if (track == 4) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, ambient4[i]);
    }
  } else if (track == 5) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, ambient5[i]);
    }
  }

  LEDControl.write();
  
}

void displaySound(){
  changeTrack = false;
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(soundsArray[track].soundName);
  display.println(" ");
  display.display();
}

void movementSound() {
  myMP3Trigger.setVolume(map(M3D3Max(M3D3Abs(currentSpeed), M3D3Abs(currentTurn)), 0, 85, 70, 1));
  if (soundTimer + soundInterval < millis()) {
    track = 6;
    myMP3Trigger.trigger(track /*+ offset*/);
    soundTimer = millis();
    soundInterval = soundsArray[track].millisecs;
    
  }
}

// =======================================================================================
//      YOUR CUSTOM DROID FUNCTIONS SHOULD END HERE
// =======================================================================================

// =======================================================================================
//      CORE DROID CONTROL FUNCTIONS START HERE - EDIT WITH CAUTION
// =======================================================================================
// Read the PS3 Controller and set request state variables
void readPS3Request()
{
     if (!extraRequestInputs) {
      
         if (PS3Controller->getButtonPress(UP))
         {              
                Serial.println("Button: UP Selected");
    
                reqArrowUp = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
                
         }
      
         if (PS3Controller->getButtonPress(DOWN))
         {
                Serial.println("Button: DOWN Selected");
    
                reqArrowDown = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
           
         }
    
         if (PS3Controller->getButtonPress(LEFT))
         {
                Serial.println("Button: LEFT Selected");
    
                reqArrowLeft = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
    
         }
         
         if (PS3Controller->getButtonPress(RIGHT))
         {
                Serial.println("Button: RIGHT Selected");
    
                reqArrowRight = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
                         
         }
         
         if (PS3Controller->getButtonPress(CIRCLE))
         {
                Serial.println("Button: CIRCLE Selected");
    
                reqCircle = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
               
         }
    
         if (PS3Controller->getButtonPress(CROSS))
         {
                Serial.println("Button: CROSS Selected");
    
                reqCross = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
                  
         }
         
         if (PS3Controller->getButtonPress(TRIANGLE))
         {
                Serial.println("Button: TRIANGLE Selected");
    
                reqTriangle = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
                  
         }
         
    
         if (PS3Controller->getButtonPress(SQUARE))
         {
                Serial.println("Button: SQUARE Selected");
    
                reqSquare = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
                  
         }
         
         if (PS3Controller->getButtonPress(L1))
         {
                Serial.println("Button: LEFT 1 Selected");
    
                reqL1 = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(L2))
         {
                Serial.println("Button: LEFT 2 Selected");
    
                reqL2 = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(R1))
         {
                Serial.println("Button: RIGHT 1 Selected");
    
                reqR1 = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(R2))
         {
                Serial.println("Button: RIGHT 2 Selected");
    
                reqR2 = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(SELECT))
         {
                Serial.println("Button: SELECT Selected");
    
                reqSelect = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(START))
         {
                Serial.println("Button: START Selected");
    
                reqStart = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(PS))
         {
                Serial.println("Button: PS Selected");
    
                reqPS = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
     }

     if (((abs(PS3Controller->getAnalogHat(LeftHatY)-128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(LeftHatX)-128) > joystickDeadZoneRange)))
     {    
            reqLeftJoyUp = false;
            reqLeftJoyDown = false;
            reqLeftJoyLeft = false;
            reqLeftJoyRight = false;
            reqLeftJoyYValue = 0;
            reqLeftJoyXValue = 0;
            reqLeftJoyMade = true;

            int currentValueY = PS3Controller->getAnalogHat(LeftHatY) - 128;
            int currentValueX = PS3Controller->getAnalogHat(LeftHatX) - 128;
            
            char yString[5];
            itoa(currentValueY, yString, 10);

            char xString[5];
            itoa(currentValueX, xString, 10);

            Serial.print("LEFT Joystick Y Value: ");
            Serial.println(yString);
            Serial.print("LEFT Joystick X Value: ");
            Serial.println(xString);

            if (currentValueY > joystickDeadZoneRange) {
                Serial.println("Left Joystick DOWN");
                reqLeftJoyDown = true;
                reqLeftJoyYValue = currentValueY;
            }

            if (currentValueY < (-1 * joystickDeadZoneRange)) {
                Serial.println("Left Joystick UP");
                reqLeftJoyUp = true;
                reqLeftJoyYValue = currentValueY;
            }

            if (currentValueX > joystickDeadZoneRange) {
                Serial.println("Left Joystick RIGHT");
                reqLeftJoyRight = true;
                reqLeftJoyXValue = currentValueX;
            }
            
            if (currentValueX < (-1 * joystickDeadZoneRange)) {
                Serial.println("Left Joystick LEFT");
                reqLeftJoyLeft = true;
                reqLeftJoyXValue = currentValueX;
            }
     } else {
          if (reqLeftJoyMade) {
              reqLeftJoyUp = false;
              reqLeftJoyDown = false;
              reqLeftJoyLeft = false;
              reqLeftJoyRight = false;
              reqLeftJoyYValue = 0;
              reqLeftJoyXValue = 0;
              reqLeftJoyMade = false;
          }
     }

     if (((abs(PS3Controller->getAnalogHat(RightHatY)-128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(RightHatX)-128) > joystickDeadZoneRange)))
     {
            reqRightJoyUp = false;
            reqRightJoyDown = false;
            reqRightJoyLeft = false;
            reqRightJoyRight = false;
            reqRightJoyYValue = 0;
            reqRightJoyXValue = 0;
            reqRightJoyMade = true;
            
            int currentValueY = PS3Controller->getAnalogHat(RightHatY) - 128;
            int currentValueX = PS3Controller->getAnalogHat(RightHatX) - 128;

            char yString[5];
            itoa(currentValueY, yString, 10);

            char xString[5];
            itoa(currentValueX, xString, 10);

            Serial.print("RIGHT Joystick Y Value: ");
            Serial.println(yString);
            Serial.print("RIGHT Joystick X Value: ");
            Serial.println(xString);

            if (currentValueY > joystickDeadZoneRange) {
                Serial.println("Right Joystick DOWN");
                reqRightJoyDown = true;
                reqRightJoyYValue = currentValueY;
            }

            if (currentValueY < (-1 * joystickDeadZoneRange)) {
                Serial.println("Right Joystick UP");
                reqRightJoyUp = true;
                reqRightJoyYValue = currentValueY;
            }

            if (currentValueX > joystickDeadZoneRange) {
                Serial.println("Right Joystick RIGHT");
                reqRightJoyRight = true;
                reqRightJoyXValue = currentValueX;
            }
            
            if (currentValueX < (-1 * joystickDeadZoneRange)) {
                Serial.println("Right Joystick LEFT");
                reqRightJoyLeft = true;
                reqRightJoyXValue = currentValueX;
            }
     } else {
          if (reqRightJoyMade) {
              reqRightJoyUp = false;
              reqRightJoyDown = false;
              reqRightJoyLeft = false;
              reqRightJoyRight = false;
              reqRightJoyYValue = 0;
              reqRightJoyXValue = 0;
              reqRightJoyMade = false;
          }
     }    
}

// Reset the PS3 request variables on every processing loop when needed
void resetRequestVariables()
{
    reqArrowUp = false;
    reqArrowDown = false;
    reqArrowLeft = false;
    reqArrowRight = false;
    reqCircle = false;
    reqCross = false;
    reqTriangle = false;
    reqSquare = false;
    reqL1 = false;
    reqL2 = false;
    reqR1 = false;
    reqR2 = false;
    reqSelect = false;
    reqStart = false;
    reqPS = false;
}

// Initialize the PS3 Controller Trying to Connect
void onInitPS3Controller()
{
    PS3Controller->setLedOn(LED1);
    isPS3ControllerInitialized = true;
    badPS3Data = 0;

    mainControllerConnected = true;
    WaitingforReconnect = true;

    Serial.println("We have the controller connected");
    Serial.print("Dongle Address: ");
    String dongle_address = String(Btd.my_bdaddr[5], HEX) + ":" + String(Btd.my_bdaddr[4], HEX) + ":" + String(Btd.my_bdaddr[3], HEX) + ":" + String(Btd.my_bdaddr[2], HEX) + ":" + String(Btd.my_bdaddr[1], HEX) + ":" + String(Btd.my_bdaddr[0], HEX);
    Serial.println(dongle_address);
}

// Determine if we are having connection problems with the PS3 Controller
boolean criticalFaultDetect()
{
    if (PS3Controller->PS3Connected)
    {
        
        currentTime = millis();
        lastMsgTime = PS3Controller->getLastMessageTime();
        msgLagTime = currentTime - lastMsgTime;            
        
        if (WaitingforReconnect)
        {   
            if (msgLagTime < 200)
            {          
                WaitingforReconnect = false;         
            }
            lastMsgTime = currentTime;    
        } 
        
        if ( currentTime >= lastMsgTime)
        {
              msgLagTime = currentTime - lastMsgTime;      
        } else
        {
             msgLagTime = 0;
        }
        
        if ( msgLagTime > 5000 )
        {
            Serial.println("It has been 5s since we heard from Controller");
            Serial.println("Disconnecting the controller");
            
            PS3Controller->disconnect();
            WaitingforReconnect = true;
            return true;
        }

        //Check PS3 Signal Data
        if(!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged))
        {
            //We don't have good data from the controller.
            //Wait 15ms - try again
            delay(15);
            Usb.Task();   
            lastMsgTime = PS3Controller->getLastMessageTime();
            
            if(!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged))
            {
                badPS3Data++;
                Serial.println("**Invalid data from PS3 Controller. - Resetting Data**");
                return true;
            }
        }
        else if (badPS3Data > 0)
        {

            badPS3Data = 0;
        }
        
        if ( badPS3Data > 10 )
        {
            Serial.println("Too much bad data coming from the PS3 Controller");
            Serial.println("Disconnecting the controller");
            
            PS3Controller->disconnect();
            WaitingforReconnect = true;
            return true;
        }
    }
    
    return false;
}

// USB Read Function - Supports Main Program Loop
boolean readUSB()
{
    Usb.Task();    
    //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
    if (PS3Controller->PS3Connected) 
    {
        if (criticalFaultDetect())
        {
            //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
            return false;
        }
        
    } 
    return true;
}
