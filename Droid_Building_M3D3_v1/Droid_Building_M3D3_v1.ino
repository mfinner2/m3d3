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
boolean servoMoving = false;
int servoPosition = 5;
boolean force = false;
long servoTimer = millis();
 int servoInterval = 4500;

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

 Sound soundsArray[12];
 long soundTimer = millis();
 int soundInterval = -1;
 int track = 0;

 bool changeTrack = false;

 bool ambientPlaying = true;
 bool initiateAmbient = false;


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
const int numLEDs = 8;
const int numLidLEDsGreen = 4;
const int numLidLEDsRed = 2;

int lidLEDsGreenOffset = numLEDs;
int lidLEDsRedOffset = numLEDs + numLidLEDsGreen;

int ambient1[numLEDs];
int ambient2[numLEDs];
int ambient3[numLEDs];
int ambient4[numLEDs];
int ambient5[numLEDs];
int ambient6[numLEDs];

boolean leftLights[(numLEDs + numLidLEDsGreen + numLidLEDsRed)] = {true, true, false, true, false, false, true, false, false, false, true, true, false, true};

// ---------------------------------------------------------------------------------------
//    Used for Infrared
// ---------------------------------------------------------------------------------------
int infraredPin = 10;
boolean infraredValue;

// ---------------------------------------------------------------------------------------
//    Used with Routines
// ---------------------------------------------------------------------------------------
boolean inRoutine1 = false;
int routine1Stage = -1;

boolean inRoutine2 = false;
int routine2Stage = -1;
int chachaFirst = 12;
//int chachaLast = 38;, 27 total chacha pieces
int currChaCha = chachaFirst - 1;

boolean inRoutine3 = false;
int routine3Stage = -1;

boolean routineStageFinished = true;
long lidTime = 0;
boolean routineServoIsUp = false;
long forBackTimer = millis();
long genTurnTimer = millis();
long routineServoTimer = millis();
boolean routineSoundTriggered = false;
boolean routineLightsTriggered = false;



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
   strcpy(soundsArray[6].soundName, "TRASH");
   soundsArray[6].millisecs = 1200;
   strcpy(soundsArray[7].soundName, "Bin Close");
   soundsArray[7].millisecs = 3000;
   strcpy(soundsArray[8].soundName, "Bin Open");
   soundsArray[8].millisecs = 3000;
   strcpy(soundsArray[9].soundName, "Baby Shark");
   soundsArray[9].millisecs = 10000;
   strcpy(soundsArray[10].soundName, "Uptown Funk");
   soundsArray[10].millisecs = 10000;
   strcpy(soundsArray[11].soundName, "Christmas");
   soundsArray[11].millisecs = 10000;

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
   for (int i = 0; i < numLEDs + numLidLEDsGreen + numLidLEDsRed; i++) {
    LEDControl.setPWM(i, 0);
   }
   LEDControl.write();

    for (int i = 0; i < numLEDs; i++) {
      ambient1[i] = (i % 2 == 0) ? ledMaxBright : 0;
      ambient2[i] = (i % 3 == 1) ? ledMaxBright : ((i % 3 == 2) ? ledMaxBright/2 : 0);
      ambient3[i] = (i % 3 == 2) ? ledMaxBright : 0;
      ambient4[i] = (i % 3 == 0) ? ledMaxBright : ((i % 2 == 0) ? ledMaxBright/3 : 0);
      ambient5[i] = (i % 2 == 1) ? ledMaxBright : 0;
      ambient6[i] = (i % 4 == 1) ? ledMaxBright : ((i % 2 == 0) ? ledMaxBright * 2 / 3 : ledMaxBright / 3);
    }

   //Infrared
   pinMode(10, INPUT);

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
            autoMode = true;
            autonCounter = -1;
            autonSetFinished = true;
            ambientPlaying = false;
       }

       if (reqCircle && !inRoutine2 && !inRoutine3) {
        inRoutine1 = true;
        routine1Stage = -1;
        routineStageFinished = true;
        routineSoundTriggered = true;
        ambientPlaying = false;
       }

       if (reqTriangle && !inRoutine1 && !inRoutine3) {
        inRoutine2 = true;
        routine2Stage = -1;
        routineStageFinished = true;
        currChaCha = chachaFirst;
        routineSoundTriggered = true;
        ambientPlaying = false;
       }

       if (reqSquare && !inRoutine1 && !inRoutine2) {
        inRoutine3 = true;
        routine3Stage = -1;
        routineStageFinished = true;
        routineSoundTriggered = true;
        ambientPlaying = false;
       }

       if (inRoutine1) {
        routine1();
       }

       if (inRoutine2) {
        routine2();
       }

       if (inRoutine3) {
        routine3();
       }

       if (autoMode) {
        autoRoutine();
       }

       
       if (!inRoutine1 && !inRoutine2 && !inRoutine3 && !autoMode) {
        if (reqSelect) {
          initiateAmbient = true;
        }
       /*if (droidMoving && ambientPlaying) {
          ambientPlaying = false;
          soundInterval = 0;
          movementSound(); 
          changeTrack = true;
        }

        if (droidMoving) {
          movementSound();
        }*/

        /*if (!droidMoving && !ambientPlaying) {
          ambientPlaying = true;
          soundInterval = 0;
        }

        readInfrared();

        if (!infraredValue) {
          servoUp();
        }

        servoDown();

        if (!servoMoving && !droidMoving) {
          ambientPlaying = true;
        }

        if (servoMoving && ambientPlaying) {
          ambientPlaying = false;
        }

        servoMoving = false;*/

        //if (ambientPlaying) {
          ambientNoise();
        //}

        //if (changeTrack) {
        //  displaySound();
        //}
        /*if (reqArrowUp) {
          force = true;
          servoUp();
       }

       if (reqArrowDown) {
          force = true;
          servoDown();
       }*/

       moveDroid();
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
    if (force || servoPosition != 165) {
    myServo.write(165);
    servoPosition = 165;
    track = 8;
    myMP3Trigger.setVolume(25);
    myMP3Trigger.trigger(track);
    soundTimer = millis();
    soundInterval = soundsArray[track].millisecs;
    changeTrack = true;
    force = false;
    servoMoving = true;
    lidUpLights();
    }
    servoTimer = millis();

}

void servoDown()
{
    //Serial.println("Droid is now executing my custom ARROW DOWN function");
    if (force || (servoTimer + servoInterval < millis() && servoPosition != 5)) {
    myServo.write(5);
    servoPosition = 5;
    track = 7;
    myMP3Trigger.setVolume(25);
    myMP3Trigger.trigger(track);
    soundTimer = millis();
    changeTrack = true;
    soundInterval = soundsArray[track].millisecs;
    force = false;
    servoMoving = true;
    lidDownLights();
    }
}

void lidUpLights() {
  for (int i = 0; i < numLEDs + numLidLEDsGreen + numLidLEDsRed; i++) {
    if (i >= lidLEDsGreenOffset && i < lidLEDsGreenOffset + numLidLEDsGreen) {
      LEDControl.setPWM(i, ledMaxBright);
    } else {
      LEDControl.setPWM(i, 0);
    }
  }

  LEDControl.write();
}

void lidDownLights() {
  for (int i = 0; i < numLEDs + numLidLEDsGreen + numLidLEDsRed; i++) {
    if (i >= lidLEDsRedOffset && i < lidLEDsRedOffset + numLidLEDsRed) {
      LEDControl.setPWM(i, ledMaxBright);
    } else {
      LEDControl.setPWM(i, 0);
    }
  }

  LEDControl.write();
}

void moveDroid() { //not finished, but a start
  if (reqLeftJoyMade || reqRightJoyMade) {
    currentSpeed = M3D3Max(M3D3Min(currentSpeed + 1, reqLeftJoyYValue * 2 / 3), currentSpeed - 1);
    currentTurn = M3D3Min(160 - M3D3Abs(currentSpeed), M3D3Max(-1 * (160 - M3D3Abs(currentSpeed)), reqRightJoyXValue * 2/3));
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

void autoRoutine() {
        if (autonSetFinished) {
          //stopDrive();
          autonCounter++;
          turnTimer = millis();
          autonSetFinished = false;
          if (autonCounter == 0) {
            sonarIntervalTimer = millis();
            currentFrontDistance = -1; //distance in centimeters
            currentLeftFrontDistance = -1;
            currentBackDistance = -1;
            currentLeftBackDistance = -1;
            tapeDistanceFront = -1;
            tapeDistanceBack = -1;
          }
        }
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
        } else if (autonCounter == 1 || autonCounter == 3 || autonCounter == 5) {
          turnRight();
        } else if (autonCounter == 7 || autonCounter == 9 || autonCounter == 11 || autonCounter == 13) {
          moveBackward();
        } else if (autonCounter == 8 || autonCounter == 10 || autonCounter == 12) {
          turnRightBackward();
        } else {
          autoMode = false;
        }
        
          /*Serial.print("Front: ");
          Serial.println(currentFrontDistance);
          Serial.print("Left Front: ");
          Serial.println(currentLeftFrontDistance);
          Serial.print("Left Back: ");
          Serial.println(currentLeftBackDistance);
          Serial.print("Back: ");
          Serial.println(currentBackDistance);*/
          /*if (autonCounter == 0 || autonCounter == 2 || autonCounter == 4 || autonCounter == 6) {
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
          }*/
        } else {
          tapeDistanceFront = currentLeftFrontDistance;
          tapeDistanceBack = currentLeftBackDistance;
        }
        //front should be about 2 less than dist from left before turn
       }

void moveForward() {
  if (currentFrontDistance - ((tapeDistanceFront + tapeDistanceBack)/2) > -1 || currentFrontDistance < 0) {
    autonSwerveVal = 0;
    if (tapeDistanceFront < currentLeftFrontDistance) {
      autonSwerveVal -= (currentLeftFrontDistance - tapeDistanceFront);
    }
    if (tapeDistanceFront > currentLeftFrontDistance) {
      autonSwerveVal += (tapeDistanceFront - currentLeftFrontDistance);
    }
    ST->turn(autonSwerveVal);
    ST->drive(-50);
  } else {
    autonSetFinished = true;
  }
  
}

void turnRight() {
  if ((turnTimer + 1000) > millis()) { //bumped up since it might be catching the wall at the last second
    ST->turn(60);
    ST->drive(0);
  } else {
    autonSetFinished = true;
  }
  
}

void moveBackward() {
  if (currentBackDistance - ((tapeDistanceFront + tapeDistanceBack)/2) > 7 || currentBackDistance < 0) { //likely increase the 7
    autonSwerveVal = 0;
    if (tapeDistanceFront < currentLeftFrontDistance) {
      autonSwerveVal -= (currentLeftFrontDistance - tapeDistanceFront); //halve
    }
    if (tapeDistanceFront > currentLeftFrontDistance) {
      autonSwerveVal += (tapeDistanceFront - currentLeftFrontDistance); //halve
    }
    ST->turn(-1 * autonSwerveVal);
    ST->drive(50);
  } else {
    autonSetFinished = true;
  }
}

void turnRightBackward() {
  if ((turnTimer + 1000) > millis()) { //bumped up since it might be catching the wall at the last second
    ST->turn(-60);
    ST->drive(0);
  } else {
    autonSetFinished = true;
  }
}

void stopDrive() {
  ST->stop();
}

void ambientNoise() {
  if (initiateAmbient) {
  if (track != 0 && soundTimer + soundsArray[track].millisecs < millis()) {
    changeTrack = true;
    track = 0;
  }
  
  if (soundTimer + soundInterval + soundsArray[track].millisecs < millis()) {
    //randomize track
    changeTrack = true;
    track = random(1, 6);
    myMP3Trigger.setVolume(25);
    myMP3Trigger.trigger(track /*+ offset*/);
    soundTimer = millis();
    soundInterval = random(1000, 5000);
    ambientLights();
    
  }
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

  for (int i = numLEDs; i < numLEDs + numLidLEDsGreen + numLidLEDsRed; i++) {
    LEDControl.setPWM(i, 0);
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

void readInfrared() {
  infraredValue = digitalRead(infraredPin);
}

void routineServoUp()
{
    if (servoPosition != 165) {
    myServo.write(165);
    servoPosition = 165;
    routineServoIsUp = true;
    track = 8;
    myMP3Trigger.setVolume(25);
    myMP3Trigger.trigger(8);
    soundTimer = millis();
    soundInterval = soundsArray[track].millisecs;
    changeTrack = true;
    servoMoving = true;
    lidTime += servoInterval;
    routineServoTimer = millis();
    lidUpLights();
    routineLightsTriggered = false;
    }

}

void routineServoDown()
{
    if (routineServoTimer + servoInterval < millis() && servoPosition != 5) {
    myServo.write(5);
    servoPosition = 5;
    routineServoIsUp = false;
    track = 7;
    myMP3Trigger.setVolume(25);
    myMP3Trigger.trigger(7);
    soundTimer = millis();
    changeTrack = true;
    soundInterval = soundsArray[track].millisecs;
    servoMoving = true;
    lidDownLights();
    }
}

void lightsAllOff() {
  if (!routineLightsTriggered) {
    for (int i = 0; i < numLEDs + numLidLEDsGreen + numLidLEDsRed; i++) {
        LEDControl.setPWM(i, 0);
    }
    LEDControl.write();
    routineLightsTriggered = true;
  }
}

void moveForBack(int vel, int dist) {
  if ((forBackTimer + lidTime + dist) > millis()) {
    ST->turn(0);
    ST->drive(vel);
  } else {
    routineStageFinished = true;
  }
}

void turnGen(int dir, int degree) {
  if ((genTurnTimer + lidTime + degree) > millis()) {
    ST->turn(dir * 60);
    ST->drive(0);
  } else {
    routineStageFinished = true;
  }
  
}

void routineSounds() {
  if (!routineSoundTriggered) {
    //randomize track
    changeTrack = true;
    track = random(1, 7);
    myMP3Trigger.setVolume(25);
    myMP3Trigger.trigger(track /*+ offset*/);
    soundTimer = millis();
    //soundInterval = random(1000, 5000);
    routineSoundTriggered = true;
    
  }
}

void routineLights() {
  if (!routineLightsTriggered) {
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
      } else if (track == 6) {
        for (int i = 0; i < numLEDs; i++) {
          LEDControl.setPWM(i, ambient6[i]);
        }
      }
    
      for (int i = numLEDs; i < numLEDs + numLidLEDsGreen; i++) {
        LEDControl.setPWM(i, 0);
      }

      for (int i = lidLEDsRedOffset; i < lidLEDsRedOffset + numLidLEDsRed; i++) {
        LEDControl.setPWM(i, ledMaxBright);
      }
    
      LEDControl.write();
      routineLightsTriggered = true;
  }
}

void routine1() {
  //meander around, intend wave to activate infrared and lid
  //when lid is opened, green light; when lid closes, red light (then turn off)

  //start in center
  if (routineStageFinished) {
    routineStageFinished = false;
    routine1Stage++;
    forBackTimer = millis();
    genTurnTimer = millis();
    lidTime = 0;
    routineSoundTriggered = false;
    routineLightsTriggered = false;
  }

  readInfrared();
  if (!infraredValue) {
    routineServoUp();
  }
  routineServoDown();
    if (!routineServoIsUp) {
    if (routine1Stage == 0) {
      moveForBack(-50, 3000);
    } else if (routine1Stage == 1) {
      turnGen(1, 1800);
    } else if (routine1Stage == 2) {
      moveForBack(-50, 2000);
    } else if (routine1Stage == 3) {
      turnGen(-1, 1500);
    } else if (routine1Stage == 4) {
      moveForBack(50, 3400);
    } else if (routine1Stage == 5) {
      turnGen(-1, 2400);
    } else if (routine1Stage == 6) {
      moveForBack(-50, 3000);
    } else if (routine1Stage == 7) {
      turnGen(-1, 600);
    } else if (routine1Stage == 8) {
      moveForBack(50, 3400);
    } else if (routine1Stage == 9) {
      turnGen(1, 1200);
    } else if (routine1Stage == 10) {
      moveForBack(-50, 2000);
    } else if (routine1Stage == 11) {
      turnGen(-1, 1400);
    } else if (routine1Stage == 12) {
      moveForBack(50, 1800);
    } else if (routine1Stage == 13) {
      turnGen(1, 600);
    } else if (routine1Stage == 14) {
      moveForBack(-50, 1400);
    } else if (routine1Stage == 15) {
      turnGen(-1, 1600);
    } else if (routine1Stage == 16) {
      moveForBack(-50, 3000);
    } else if (routine1Stage == 17) {
      turnGen(-1, 1500);
    } else if (routine1Stage == 18) {
      moveForBack(-50, 2600);
    } else if (routine1Stage == 19) {
      turnGen(1, 4500);
    } else {
      inRoutine1 = false;
      lightsAllOff();
    }
    routineSounds();
    routineLights();
  }
  
}

void chachaSounds() {
  if (!routineSoundTriggered) {
    myMP3Trigger.setVolume(25);
    myMP3Trigger.trigger(currChaCha);
    routineSoundTriggered = true;
  }
}

void crisscrossLights(long len) {
  //need something special for criss cross
  if ((forBackTimer + (len * 1 / 8)) > millis()) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, 0);
    }
    for (int i = numLEDs; i < numLEDs + numLidLEDsGreen; i++) {
      LEDControl.setPWM(i, 0);
    }

    for (int i = lidLEDsRedOffset; i < lidLEDsRedOffset + numLidLEDsRed; i++) {
      LEDControl.setPWM(i, ledMaxBright);
    }
  } else if ((forBackTimer + (len * 2 / 8)) > millis()) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, 0);
    }
    for (int i = numLEDs; i < numLEDs + numLidLEDsGreen; i++) {
      LEDControl.setPWM(i, ledMaxBright);
    }

    for (int i = lidLEDsRedOffset; i < lidLEDsRedOffset + numLidLEDsRed; i++) {
      LEDControl.setPWM(i, 0);
    }
  } else if ((forBackTimer + (len * 3 / 8)) > millis()) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, 0);
    }
    for (int i = numLEDs; i < numLEDs + numLidLEDsGreen; i++) {
      LEDControl.setPWM(i, 0);
    }

    for (int i = lidLEDsRedOffset; i < lidLEDsRedOffset + numLidLEDsRed; i++) {
      LEDControl.setPWM(i, ledMaxBright);
    }
  } else if ((forBackTimer + (len * 4 / 8)) > millis()) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, 0);
    }
    for (int i = numLEDs; i < numLEDs + numLidLEDsGreen; i++) {
      LEDControl.setPWM(i, ledMaxBright);
    }

    for (int i = lidLEDsRedOffset; i < lidLEDsRedOffset + numLidLEDsRed; i++) {
      LEDControl.setPWM(i, 0);
    }
  } else if ((forBackTimer + (len * 5 / 8)) > millis()) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, 0);
    }
    for (int i = numLEDs; i < numLEDs + numLidLEDsGreen; i++) {
      LEDControl.setPWM(i, 0);
    }

    for (int i = lidLEDsRedOffset; i < lidLEDsRedOffset + numLidLEDsRed; i++) {
      LEDControl.setPWM(i, ledMaxBright);
    }
  } else if ((forBackTimer + (len * 6 / 8)) > millis()) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, 0);
    }
    for (int i = numLEDs; i < numLEDs + numLidLEDsGreen; i++) {
      LEDControl.setPWM(i, ledMaxBright);
    }

    for (int i = lidLEDsRedOffset; i < lidLEDsRedOffset + numLidLEDsRed; i++) {
      LEDControl.setPWM(i, 0);
    }
  } else if ((forBackTimer + (len * 7 / 8)) > millis()) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, 0);
    }
    for (int i = numLEDs; i < numLEDs + numLidLEDsGreen; i++) {
      LEDControl.setPWM(i, 0);
    }

    for (int i = lidLEDsRedOffset; i < lidLEDsRedOffset + numLidLEDsRed; i++) {
      LEDControl.setPWM(i, ledMaxBright);
    }
  } else if ((forBackTimer + (len * 8 / 8)) > millis()) {
    for (int i = 0; i < numLEDs; i++) {
      LEDControl.setPWM(i, 0);
    }
    for (int i = numLEDs; i < numLEDs + numLidLEDsGreen; i++) {
      LEDControl.setPWM(i, ledMaxBright);
    }

    for (int i = lidLEDsRedOffset; i < lidLEDsRedOffset + numLidLEDsRed; i++) {
      LEDControl.setPWM(i, 0);
    }
  } else {
    routineStageFinished = true;
  }
  LEDControl.write();
}

void chachaServo(int len) {
  if (routineServoTimer + (len / 2) > millis()) {
    if (servoPosition != 165) {
      myServo.write(165);
      servoPosition = 165;
    }
  } else if (routineServoTimer + len > millis()) {
    if (servoPosition != 5) {
      myServo.write(5);
      servoPosition = 5;
    }
  }
  else {
    routineStageFinished = true;
  }
}

void moveChaCha(int vel, int dist) {
  if ((forBackTimer + (dist * 9 / 10)) > millis()) {
    ST->turn(0);
    ST->drive(vel);
  } else if ((forBackTimer + dist) > millis()) {
    ST->stop();
  }
  else {
    routineStageFinished = true;
  }
}

void turnChaCha(int dir, long len) {
  if ((genTurnTimer + (len * 9 / 10)) > millis()) {
    ST->turn(dir * 60);
    ST->drive(0);
  } else if ((genTurnTimer + len) > millis()) {
    ST->stop();
  } else {
    routineStageFinished = true;
  }
  
}

void chachaStomp(int dir, long len) {
  if ((genTurnTimer + (len * 5 / 11)) > millis()) {
    ST->turn(dir * 60);
    ST->drive(0);
  } else if ((genTurnTimer + (len * 10 / 11)) > millis()) {
    ST->turn(dir * -1 * 60);
    ST->drive(0);
  } else if ((genTurnTimer + len) > millis()) {
    ST->stop();
  } else {
    routineStageFinished = true;
  }
}

void chachaLeft() {
  if (!routineLightsTriggered) {
    for (int i = 0; i < numLEDs + numLidLEDsGreen + numLidLEDsRed; i++) {
      if (leftLights[i]) {
      LEDControl.setPWM(i, ledMaxBright);
      } else {
        LEDControl.setPWM(i, 0);
      }
    }
    LEDControl.write();
    routineLightsTriggered = true;
  }
}

void chachaRight() {
  if (!routineLightsTriggered) {
    for (int i = 0; i < numLEDs + numLidLEDsGreen + numLidLEDsRed; i++) {
      if (leftLights[i]) {
      LEDControl.setPWM(i, 0);
      } else {
        LEDControl.setPWM(i, ledMaxBright);
      }
    }
    LEDControl.write();
    routineLightsTriggered = true;
  }
}

void chachaAllOn() {
  if (!routineLightsTriggered) {
    for (int i = 0; i < numLEDs + numLidLEDsGreen + numLidLEDsRed; i++) {
        LEDControl.setPWM(i, ledMaxBright);
    }
    LEDControl.write();
    routineLightsTriggered = true;
  }
}

void routine2() {
  //cha cha slide
  //start in center
  if (routineStageFinished) {
    routineStageFinished = false;
    routine2Stage++;
    if (routine2Stage == 0){
      routineSoundTriggered = false;
    }
    forBackTimer = millis();
    genTurnTimer = millis();
    routineServoTimer = millis();
    routineLightsTriggered = false;
  }

  chachaSounds();

  if (routine2Stage == 0) {
    if (forBackTimer + 2800 < millis()) {
      routineStageFinished = true;
    }
    chachaAllOn();
  } else if (routine2Stage == 1) {
    turnChaCha(-1, 2240);
    chachaLeft();
    //to the left, 2240
    
  } else if (routine2Stage == 2) {
    moveChaCha(50, 1600);
    chachaAllOn();
    //take it back, 1200
  } else if (routine2Stage == 3) {
    chachaServo(2000);
    chachaAllOn();
    //one hop, 2400
  } else if (routine2Stage == 4) {
    chachaStomp(1, 1600);
    chachaRight();
    //right foot stomp, 1600
    
  } else if (routine2Stage == 5) {
    chachaStomp(-1, 1960);
    chachaLeft();
    //left foot stomp, 1960
    
  } else if (routine2Stage == 6) {
    //chachaStomp(1, 1600);
    //turnChaCha(-1, 6380);
    turnChaCha(1, 6380);
    chachaRight();
    //cha cha, 1880
    
  } else if (routine2Stage == 7) {
    //to the left, 1480
    turnChaCha(-1, 1480);
    chachaLeft();
  } else if (routine2Stage == 8) {
    //take it back, 2200
    moveChaCha(50, 2200);
    chachaAllOn();
    
  } else if (routine2Stage == 9) {
    //one hop
    chachaServo(2400);
    chachaAllOn();
    
  } else if (routine2Stage == 10) {
    //right foot stomp
    chachaStomp(1, 1600);
    chachaRight();
    
  } else if (routine2Stage == 11) {
    //left foot stomp
    chachaStomp(-1, 1960);
    chachaLeft();
    
  } else if (routine2Stage == 12) {
    //cha cha
    turnChaCha(-1, 4000);
    chachaLeft();
    
  } else if (routine2Stage == 13) {
    //to the right
    turnChaCha(1, 2240);
    chachaRight();
    
  } else if (routine2Stage == 14) {
    //to the left
    turnChaCha(-1, 1840);
    chachaLeft();
    
  } else if (routine2Stage == 15) {
    //take it back
    moveChaCha(50, 1600);
    chachaAllOn();
  } else if (routine2Stage == 16) {
    //one hop
    chachaServo(2400);
  } else if (routine2Stage == 17) {
    //one hop
    chachaServo(2000);
    chachaAllOn();
  } else if (routine2Stage == 18) {
    //right foot stomp x2
    chachaStomp(1, 850);
    chachaRight();
  } else if (routine2Stage == 19) {
    //right foot stomp x2
    chachaStomp(1, 850);
    chachaRight();
  } else if (routine2Stage == 20) {
    //left foot stomp x2
    chachaStomp(-1, 850);
    chachaLeft();
  } else if (routine2Stage == 21) {
    //left foot stomp x2
    chachaStomp(-1, 850);
    chachaLeft();
  } else if (routine2Stage == 22) {
    //slide left
    turnChaCha(-1, 2240);
    chachaLeft();
    
    
  } else if (routine2Stage == 23) {
    //slide right
    turnChaCha(1, 2240);
    chachaRight();
    
  } else if (routine2Stage == 24) {
    //criss cross x2
    crisscrossLights(4000);
    if (forBackTimer + 4000 < millis()) {
      routineStageFinished = true;
    }
  } else if (routine2Stage == 25) {
    //cha cha
    turnChaCha(1, 2500);
    chachaRight();
  } else {
    inRoutine2 = false;
    lightsAllOff();
  }
  //chachaLights();
}

void chomp() {
  if (routineServoTimer + 500 > millis()) {
    if (servoPosition != 165) {
      myServo.write(165);
      servoPosition = 165;
    }
  } else if (routineServoTimer + 1000 > millis()) {
    if (servoPosition != 5) {
      myServo.write(5);
      servoPosition = 5;
    }
  } else {
    routineServoTimer = millis();
  }
}

void getOutSounds() {
  if (!routineSoundTriggered) {
    myMP3Trigger.setVolume(25);
    myMP3Trigger.trigger(9);
    routineSoundTriggered = true;
  }
}

void getOutLights() {
  if (servoPosition == 165) {
    if (!routineLightsTriggered) {
      for (int i = 0; i < numLEDs + numLidLEDsGreen + numLidLEDsRed; i++) {
          LEDControl.setPWM(i, ledMaxBright);
      }
      LEDControl.write();
      routineLightsTriggered = true;
    }
  } else if (servoPosition == 5) {
    if (routineLightsTriggered) {
      for (int i = 0; i < numLEDs + numLidLEDsGreen + numLidLEDsRed; i++) {
          LEDControl.setPWM(i, 0);
      }
      LEDControl.write();
      routineLightsTriggered = false;
    }
  }
}

void routine3() {
  //baby shark get out
  if (routineStageFinished) {
    routineStageFinished = false;
    routine3Stage++;
    if (routine3Stage == 0) {
      routineSoundTriggered = false;
      routineServoTimer = millis();
      routineLightsTriggered = false;
    }
    forBackTimer = millis();
    genTurnTimer = millis();
  }

  //servo stuff
  chomp();
  getOutLights();
  getOutSounds();
  //movement
  if (routine3Stage == 0) {
    turnGen(1, 1000);
  } else if (routine3Stage == 1) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 2) {
    turnGen(-1, 500);
  } else if (routine3Stage == 3) {
    turnGen(1, 500);
  } else if (routine3Stage == 4) {
    turnGen(-1, 500);
  } else if (routine3Stage == 5) {
    turnGen(1, 500);
  } else if (routine3Stage == 6) {
    moveForBack(70, 500);
  } else if (routine3Stage == 7) {
    turnGen(1, 1000);
  } else if (routine3Stage == 8) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 9) {
    turnGen(-1, 500);
  } else if (routine3Stage == 10) {
    turnGen(1, 500);
  } else if (routine3Stage == 11) {
    turnGen(-1, 500);
  } else if (routine3Stage == 12) {
    turnGen(1, 500);
  } else if (routine3Stage == 13) {
    moveForBack(70, 500);
  } else if (routine3Stage == 14) {
    turnGen(1, 1000);
  } else if (routine3Stage == 15) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 16) {
    turnGen(-1, 500);
  } else if (routine3Stage == 17) {
    turnGen(1, 500);
  } else if (routine3Stage == 18) {
    turnGen(-1, 500);
  } else if (routine3Stage == 19) {
    turnGen(1, 500);
  } else if (routine3Stage == 20) {
    moveForBack(70, 500);
  } else if (routine3Stage == 21) {
    turnGen(1, 1000);
  } else if (routine3Stage == 22) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 23) {
    turnGen(-1, 500);
  } else if (routine3Stage == 24) {
    turnGen(1, 500);
  } else if (routine3Stage == 25) {
    turnGen(-1, 500);
  } else if (routine3Stage == 26) {
    turnGen(1, 500);
  } else if (routine3Stage == 27) {
    moveForBack(70, 500);
  } else if (routine3Stage == 28) {
    turnGen(1, 1000);
  } else if (routine3Stage == 29) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 30) {
    turnGen(-1, 500);
  } else if (routine3Stage == 31) {
    turnGen(1, 500);
  } else if (routine3Stage == 32) {
    turnGen(-1, 500);
  } else if (routine3Stage == 33) {
    turnGen(1, 500);
  } else if (routine3Stage == 34) {
    moveForBack(70, 500);
  } else if (routine3Stage == 35) {
    turnGen(1, 1000);
  } else if (routine3Stage == 36) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 37) {
    turnGen(-1, 500);
  } else if (routine3Stage == 38) {
    turnGen(1, 500);
  } else if (routine3Stage == 39) {
    turnGen(-1, 500);
  } else if (routine3Stage == 40) {
    turnGen(1, 500);
  } else if (routine3Stage == 41) {
    moveForBack(70, 500);
  } else if (routine3Stage == 42) {
    turnGen(1, 1000);
  } else if (routine3Stage == 43) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 44) {
    turnGen(-1, 500);
  } else if (routine3Stage == 45) {
    turnGen(1, 500);
  } else if (routine3Stage == 46) {
    turnGen(-1, 500);
  } else if (routine3Stage == 47) {
    turnGen(1, 500);
  } else if (routine3Stage == 48) {
    moveForBack(70, 500);
  } else if (routine3Stage == 49) {
    turnGen(1, 1000);
  } else if (routine3Stage == 50) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 51) {
    turnGen(-1, 500);
  } else if (routine3Stage == 52) {
    turnGen(1, 500);
  } else if (routine3Stage == 53) {
    turnGen(-1, 500);
  } else if (routine3Stage == 54) {
    turnGen(1, 500);
  } else if (routine3Stage == 55) {
    moveForBack(70, 500);
  } else if (routine3Stage == 56) {
    turnGen(1, 1000);
  } else if (routine3Stage == 57) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 58) {
    turnGen(-1, 500);
  } else if (routine3Stage == 59) {
    turnGen(1, 500);
  } else if (routine3Stage == 60) {
    turnGen(-1, 500);
  } else if (routine3Stage == 61) {
    turnGen(1, 500);
  } else if (routine3Stage == 62) {
    moveForBack(70, 500);
  } else if (routine3Stage == 63) {
    turnGen(1, 1000);
  } else if (routine3Stage == 64) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 65) {
    turnGen(-1, 500);
  } else if (routine3Stage == 66) {
    turnGen(1, 500);
  } else if (routine3Stage == 67) {
    turnGen(-1, 500);
  } else if (routine3Stage == 68) {
    turnGen(1, 500);
  } else if (routine3Stage == 69) {
    moveForBack(70, 500);
  } else if (routine3Stage == 70) {
    turnGen(1, 1000);
  } else if (routine3Stage == 71) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 72) {
    turnGen(-1, 500);
  } else if (routine3Stage == 73) {
    turnGen(1, 500);
  } else if (routine3Stage == 74) {
    turnGen(-1, 500);
  } else if (routine3Stage == 75) {
    turnGen(1, 500);
  } else if (routine3Stage == 76) {
    moveForBack(70, 500);
  } else if (routine3Stage == 77) {
    turnGen(1, 1000);
  } else if (routine3Stage == 78) {
    moveForBack(-70, 500);
  } else if (routine3Stage == 79) {
    turnGen(-1, 500);
  } else if (routine3Stage == 80) {
    turnGen(1, 500);
  } else if (routine3Stage == 81) {
    turnGen(-1, 500);
  } else if (routine3Stage == 82) {
    turnGen(1, 500);
  } else if (routine3Stage == 83) {
    moveForBack(70, 500);
  } else {
    inRoutine3 = false;
    force = true;
    servoDown();
    lightsAllOff();
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
