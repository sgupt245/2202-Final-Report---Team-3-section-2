// Robot Climbing Code
//138 - 161: variable set-up  New Veriables for Paper lengths have been created.
//233 - 242: reset protocols (when the override switch is pressed)
//293 - 541: combined code (mostly small modifications to exercise 3 and 1 of lab 5)
//417 - 499: Exercise 1 code of lab 5 - ideally for the beacon navigation 
//656 - 666: Roller state function
//MSE 2202
//Western Engineering base code
//2020 05 13 E J Porter
/*
  esp32                                           MSE-DuinoV2
  pins         description                        Brd Jumpers /Labels                                                                  User (Fill in chart with user PIN usage)
  1             3v3                               PWR 3V3                                                                              3V3
  2             gnd                               GND                                                                                  GND
  3             GPIO15/AD2_3/T3/SD_CMD/           D15 (has connections in both 5V and 3V areas)
  4             GPIO2/AD2_2/T2/SD_D0              D2(has connections in both 5V and 3V areas)  /INDICATORLED ( On ESP32 board )        Heartbeat LED
  5             GPIO4/AD2_0/T0/SD_D1              D4(has connections in both 5V and 3V areas)                                          Left Motor, Channel A
  6             GPIO16/RX2                        Slide Switch S1b                                                                     IR Receiver
  7             GPIO17/TX2                        Slide Switch S2b                                                                     Left Encoder, Channel A
  8             GPIO5                             D5 (has connections in both 5V and 3V areas)                                         Left Encoder, Channel B
  9             GPIO18                            D18 (has connections in both 5V and 3V areas)                                        Left Motor, Channel B
  10            GPIO19/CTS0                       D19 (has connections in both 5V and 3V areas)                                        Right Motor, Channel A
  11            GPIO21                            D21/I2C_DA
  12            GPIO3/RX0                         RX0
  13            GPIO1//TX0                        TX0
  14            GPIO22/RTS1                       D22/I2C_CLK
  15            GPIO23                            D23 (has connections in both 5V and 3V areas)
  16            EN                                JP4 (Labeled - RST) for reseting ESP32
  17            GPI36/VP/AD1_0                    AD0
  18            GPI39/VN/AD1_3/                   AD3
  19            GPI34/AD1_6/                      AD6
  20            GPI35/AD1_7                       Potentiometer R2 / AD7
  21            GPIO32/AD1_4/T9                   Potentiometer R1 / AD4                                                               Pot 1 (R1)
  22            GPIO33/AD1_5/T8                   IMon/D33  monitor board current
  23            GPIO25/AD2_8/DAC1                 SK6812 Smart LEDs / D25                                                              Smart LEDs
  24            GPIO26/A2_9/DAC2                  Push Button PB2                                                                      Limit switch
  25            GPIO27/AD2_7/T7                   Push Button PB1                                                                      PB1
  26            GPOP14/AD2_6/T6/SD_CLK            Slide Switch S2a                                                                     Right Encoder, Channel A
  27            GPIO12/AD2_5/T5/SD_D2/            D12(has connections in both 5V and 3V areas)                                         Right Motor, Channel B
  28            GPIO13/AD2_4/T4/SD_D3/            Slide Switch S1a                                                                     Right Encoder, Channel B
  29            GND                               GND                                                                                  GND
  30            VIN                               PWR 5V t 7V                                                                          PWR 5V to 7V
*/


//Pin assignments
const int ciHeartbeatLED = 2;
const int ciPB1 = 27;
const int ciPB2 = 26;
const int ciPot1 = A4;    //GPIO 32  - when JP2 has jumper installed Analog pin AD4 is connected to Poteniometer R1
const int ciLimitSwitch = 26;
const int ciIRDetector = 16;
const int ciMotorLeftA = 4;
const int ciMotorLeftB = 18;
const int ciMotorRightA = 19;
const int ciMotorRightB = 12;
const int ciEncoderLeftA = 17;
const int ciEncoderLeftB = 5;
const int ciEncoderRightA = 14;
const int ciEncoderRightB = 13;
const int ciSmartLED = 25;
const int ciStepperMotorDir = 22;
const int ciStepperMotorStep = 21;
const int rollerDriver = 32;
const int limitSwitch = 10;

int rollerState = 0;

volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

#include "0_Core_Zero.h"

#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>
#include <Math.h>
#include "Motion.h";
#include "MyWEBserver.h"
#include "BreakPoint.h"
#include "WDT.h";

void loopWEBServerButtonresponce(void);

const int CR1_ciMainTimer =  1000;
const int CR1_ciHeartbeatInterval = 500;
int CR1_ciMotorRunTime = 500;
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;

const uint8_t ci8RightTurn = 30;
const uint8_t ci8LeftTurn = 12;

unsigned char CR1_ucMainTimerCaseCore1;
uint8_t CR1_ui8LimitSwitch;

uint8_t CR1_ui8IRDatum;
uint8_t CR1_ui8WheelSpeed;
uint8_t CR1_ui8LeftWheelSpeed;
uint8_t CR1_ui8RightWheelSpeed;

uint32_t CR1_u32Now;
uint32_t CR1_u32Last;
uint32_t CR1_u32Temp;
uint32_t CR1_u32Avg;

unsigned long CR1_ulLastDebounceTime;
unsigned long CR1_ulLastByteTime;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = 0;

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;

boolean btHeartbeat = true;
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;

// Declare our SK6812 SMART LED object:
Adafruit_NeoPixel SmartLEDs(2, 25, NEO_GRB + NEO_KHZ400);
// Argument 1 = Number of LEDs (pixels) in use
// Argument 2 = ESP32 pin number
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

//**************************************************************************************************************************************

int ledColor; //stores the current color of the led
unsigned long beaconTrip; //used to record when beacon trips
int turnSequence = 0; //used to transition from different phases of the code
int phase = 0; //separates the preset phase with exercise 1 code
int motorIndex = 0;

int tileL = 32;   //original: 34 37
int addDist = 0;

int lTurn = 4;//original: 23                                                                                              created now
int rTurn = 17;//original: 20       test:30                                                                                      created now
//variables not used but still exist
int purpleCounter;
int redCounter;
int greenCounter;



void setup() {


  //***************************************************************************************************************************************

  Serial.begin(115200);
  Serial2.begin(2400, SERIAL_8N1, ciIRDetector);  // IRDetector on RX2 receiving 8-bit words at 2400 baud

  Core_ZEROInit();

  WDT_EnableFastWatchDogCore1();
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[0] = 0;
  WDT_vfFastWDTWarningCore1[1] = 0;
  WDT_vfFastWDTWarningCore1[2] = 0;
  WDT_vfFastWDTWarningCore1[3] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[4] = 0;
  WDT_vfFastWDTWarningCore1[5] = 0;
  WDT_vfFastWDTWarningCore1[6] = 0;
  WDT_vfFastWDTWarningCore1[7] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[8] = 0;
  WDT_vfFastWDTWarningCore1[9] = 0;
  WDT_ResetCore1();

  setupMotion();
  pinMode(ciHeartbeatLED, OUTPUT);
  pinMode(ciPB1, INPUT_PULLUP);
  pinMode(ciLimitSwitch, INPUT_PULLUP);
  pinMode(rollerDriver, OUTPUT);
  pinMode(limitSwitch, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(limitSwitch), Roller, RISING); //attach interrupt to limit switch pin to change variable when switch is pressed

  SmartLEDs.begin();                          // Initialize Smart LEDs object (required)
  SmartLEDs.clear();                          // Set all pixel colours to off
  SmartLEDs.show();                           // Send the updated pixel colours to the hardware
}

void loop()
{
  //WSVR_BreakPoint(1);



  Serial.println(beaconTrip);

  //average the encoder tick times
  ENC_Averaging();

  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
    CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
  }

  if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
      iButtonState = iButtonValue;               // update current button state

      // only toggle the run condition if the new button state is LOW
      if (iButtonState == LOW)
      {
        ENC_ClearLeftOdometer();
        ENC_ClearRightOdometer();
        btRun = !btRun;
        Serial.println(btRun);
        // if stopping, reset motor states and stop motors
        if (!btRun)
        {
          ucMotorStateIndex = 0;
          ucMotorState = 0;
          move(0);


          //********************************************************************************************************************************************
          beaconTrip = 0;//resetting all pins once the button is pressed
          turnSequence = 0;
          phase = 0;
          motorIndex = 0;
          digitalWrite(mPin10, LOW);
          digitalWrite(mPin11, LOW);


          //********************************************************************************************************************************************
        }

      }
    }
  }
  iLastButtonState = iButtonValue;             // store button state

  if (!digitalRead(ciLimitSwitch))
  {
    btRun = 0; //if limit switch is pressed stop bot
    ucMotorStateIndex = 0;
    ucMotorState = 0;
    move(0);
  }

  if (Serial2.available() > 0) {               // check for incoming data
    CR1_ui8IRDatum = Serial2.read();          // read the incoming byte
    // Serial.println(iIncomingByte, HEX);        // uncomment to output received character
    CR1_ulLastByteTime = millis();            // capture time last byte was received
  }
  else
  {
    // check to see if elapsed time exceeds allowable timeout
    if (millis() - CR1_ulLastByteTime > CR1_clReadTimeout) {
      CR1_ui8IRDatum = 0;                     // if so, clear incoming byte
    }
  }
  CR1_ulMainTimerNow = micros();
  if (CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer)
  {
    WDT_ResetCore1();
    WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;

    CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;

    switch (CR1_ucMainTimerCaseCore1) //full switch run through is 1mS
    {
      //###############################################################################
      case 0:
        {

          if (btRun)
          {
            CR1_ulMotorTimerNow = millis();
            if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime)
            {
              CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;



              //*******************************************************************************************************************************

              switch (phase) {   // phase switches the automated version to exercise 1 code
                case 0:
                  {
                    CR1_ciMotorRunTime = 1500; // keeping a set timer for all of the preset motion
                    switch (motorIndex) { // preset commands to navigate around the obstacle
                      case 0: // stops to rest
                        {
                          ucMotorState = 0;
                          move(0);
                          motorIndex++;
                          break;
                        }
                      case 1: // stops to rest
                        {
                          ucMotorState = 0;
                          move(0);
                          motorIndex++;
                          break;
                        }

                      case 2: { //goes forward

                          ENC_SetDistance((tileL * 9/2), (tileL * 9/2)); //original multiplier = 3
                          ucMotorState = 1;
                          CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                          CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                          ledcWrite(servoChannela, 7500); //old flag position, sets at 7500 in the first loop
                          motorIndex++;                                                                                 //change for later
                          break;
                        }
                      case 3: // stops to rest
                        {
                          ucMotorState = 0;
                          move(0);
                          motorIndex++;
                          break;
                        }
                      case 4: { // turns right 90 degrees
                          ENC_SetDistance(rTurn, -rTurn);
                          ucMotorState = 3;
                          CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                          CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                          motorIndex++;
                          break;
                        }
                      case 5: { //stops to rest
                          ucMotorState = 0;
                          move(0);
                          motorIndex++;
                          break;
                        }
                      case 6: { // Goes forward
                          ENC_SetDistance((tileL * (4)), (tileL * (4)));
                          ucMotorState = 1;
                          CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                          CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                          motorIndex++;
                          break;
                        }
                      case 7: { //stops to rest
                          ucMotorState = 0;
                          move(0);
                          motorIndex++;
                          break;
                        }
                      case 8: { //turns right by 90 degrees
                          ENC_SetDistance(rTurn, -rTurn);
                          ucMotorState = 3;
                          CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                          CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                          motorIndex++;
                          
                          break;
                        }
                      case 9: { //stops to rest
                          ucMotorState = 0;
                          move(0);
                          motorIndex++;
                          break;
                        }
                      case 10: {//Goes Forward
                          ENC_SetDistance((tileL * 9/2 + addDist), (tileL * 9/2 + addDist));
                          ucMotorState = 1;
                          CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                          CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                          motorIndex++;
                          break;

                        }
                      case 11: { //stops to rest
                          ucMotorState = 0;
                          move(0);
                          motorIndex++;
                         
                          break;
                          
                          
                           
                        }
                        case 12: {//Goes Forward
                          ENC_SetDistance((tileL * 1/2 ), (tileL * 1/2));
                          ucMotorState = 1;
                          CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                          CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                          motorIndex++;
                          if(rollerState == 1){
                           digitalWrite(rollerDriver, HIGH); //Writes an output of high to the mosfet gate to start the roller
                          }else{
                            digitalWrite(rollerDriver, LOW);
                          }
                          break;

                        }
                        case 13: { //stops to rest
                          ucMotorState = 0;
                          move(0);

                          
                          motorIndex ++; //failsafe: ensures that no cases repeat after executing once
                           break;
                        }
                        case 14: {//Goes Forward
                          ENC_SetDistance((tileL * 1/2 ), (tileL * 1/2));
                          ucMotorState = 1;
                          CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                          CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                          motorIndex++;
                          digitalWrite(rollerDriver, HIGH); 
                          break;

                        }
                        case 15: { //stops to rest
                          ucMotorState = 0;
                          move(0);

                          
                          motorIndex = 20; //failsafe: ensures that no cases repeat after executing once
                          phase++;
                           break;
                        }


                        //at this point, the robot is still facing the direction that it travelled in. Ideally, at the point, exercise 1 code is designed to keep
                        //the bot turning till it finds the beacon. At that point it will go in the beacon's direction. After hitting the limit switch, it will go back
                        // the backward motion will be altered to go about 1.5 sheets of paper (vertical) at the decision point
                        //This is assuming that the bot does not need to go all the way down (the description states that it just needs to return to the decision point)
                        //At that point, it will turn 180 degrees and then raise flag (as normal)



                    } // end of motorIndex
                    break;
                  } // end of case 0 of phase


                case 1: //case 1 of phase: lab 1 code begins (red = spin, green = move forward, purple = beaconTrips and does other stuff
                  {


                    switch (ledColor)
                    {
                      case 0: //Green case (supposed to go straight here)
                        {

                          if (beaconTrip == 0) { // if beacon hasn't tripped, execute this line of code

                            CR1_ciMotorRunTime = 500; //faster interval for straight line
                            ENC_SetDistance(20, 20); // most consistent set of numbers out of all (still produces different patterns every time though)
                            ucMotorState = 1;   //forward
                            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                            
                            break;
                          }
                        }

                      case 1: //red case (supposed to turn small amounts to recalibrate direction)
                        {
                          if (beaconTrip == 0) {  // if beacon hasn't tripped, execute this line of code
                            CR1_ciMotorRunTime = 1000; //slower interval for calibration so that it does not skip over when green is detected
                            ENC_SetDistance(3, -3); //trying to make it turn as little as possible (any lower makes it spin infinitely
                            ucMotorState = 3;
                            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                            

                            break;
                          }
                        }


                      case 2:
                        {
                          beaconTrip = 1; // Beacon limit switch has been pressed
                          if (beaconTrip == 1 && turnSequence == 0) { //when the switch is pressed
                            CR1_ciMotorRunTime = 1000; //slower interval for execution
                            ucMotorState = 4;  //reverse
                            ENC_SetDistance((vPaperL * 3 / 2), (vPaperL * 3 / 2)); //1.5 paper length Vertically
                            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                            beaconTrip = CR1_ulMotorTimerNow; //making it so that this code only executes once and also doing the next part
                            turnSequence  = 1;
                          }


                          break;

                        }

                    }  //end of switch statement of ledColor (all three cases: green, red, purple

                    if (((CR1_ulMotorTimerNow - beaconTrip) >= CR1_ciMotorRunTime) && turnSequence == 1 ) //executes once after going reverse to stop robot
                    {
                      CR1_ciMotorRunTime = 1000; //slower interval for execution
                      ucMotorState = 0;
                      move(0);
                      turnSequence = 2;
                      beaconTrip = CR1_ulMotorTimerNow;
                    }


                    if (((CR1_ulMotorTimerNow - beaconTrip) >= CR1_ciMotorRunTime) && turnSequence == 2 ) //180 degree turn
                    {
                      CR1_ciMotorRunTime = 1000; //slower interval for execution
                      ENC_SetDistance(43, -43); //supposed to make it turn 180 degrees
                      ucMotorState = 3;
                      CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                      CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                      
                      turnSequence = 3;
                      beaconTrip = CR1_ulMotorTimerNow;
                    }

                    if (((CR1_ulMotorTimerNow - beaconTrip) >= CR1_ciMotorRunTime) && turnSequence == 3 ) //stops robot, raises flag, executes infinitely after
                    {
                      CR1_ciMotorRunTime = 1000; //slower interval for execution
                      ucMotorState = 0;
                      move(0);
                      

                    }
                    break;
                  }// end of case 1
              }// end of phase switch statement


              //******************************************************************************************************************************




            }
          }
          CR1_ucMainTimerCaseCore1 = 1;

          break;
        }
      //###############################################################################
      case 1:
        {
          //read pot 1 for motor speeds
          // CR1_ui8WheelSpeed = map(analogRead(ciPot1), 0, 4096, 130, 255);  // adjust to range that will produce motion
          CR1_ui8WheelSpeed = 250;
          CR1_ucMainTimerCaseCore1 = 2;
          break;
        }
      //###############################################################################
      case 2:
        {
          // asm volatile("esync; rsr %0,ccount":"=a" (vui32test1)); // @ 240mHz clock each tick is ~4nS

          //   asm volatile("esync; rsr %0,ccount":"=a" (vui32test2)); // @ 240mHz clock each tick is ~4nS

          CR1_ucMainTimerCaseCore1 = 3;
          break;
        }
      //###############################################################################
      case 3:
        {
          //move bot X number of odometer ticks
          if (ENC_ISMotorRunning())
          {
            MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed, CR1_ui8LeftWheelSpeed);
          }

          CR1_ucMainTimerCaseCore1 = 4;
          break;
        }
      //###############################################################################
      case 4:
        {

          CR1_ucMainTimerCaseCore1 = 5;
          break;
        }
      //###############################################################################
      case 5:
        {


          CR1_ucMainTimerCaseCore1 = 6;
          break;
        }
      //###############################################################################
      case 6:
        {


          CR1_ucMainTimerCaseCore1 = 7;
          break;
        }
      //###############################################################################
      case 7:
        {
          if (CR1_ui8IRDatum == 0x55) {                // if proper character is seen
            SmartLEDs.setPixelColor(0, 0, 25, 0);      // make LED1 green with 10% intensity
            ledColor = 1;
          }
          else if (CR1_ui8IRDatum == 0x41) {           // if "hit" character is seen
            SmartLEDs.setPixelColor(0, 25, 0, 25);     // make LED1 purple with 10% intensity
            ledColor = 2;
          }
          else {                                       // otherwise
            SmartLEDs.setPixelColor(0, 25, 0, 0);      // make LED1 red with 10% intensity
            ledColor = 0;
          }
          SmartLEDs.show();                            // send updated colour to LEDs

          CR1_ucMainTimerCaseCore1 = 8;
          break;
        }
      //###############################################################################
      case 8:
        {

          CR1_ucMainTimerCaseCore1 = 9;
          break;
        }
      //###############################################################################
      case 9:
        {

          CR1_ucMainTimerCaseCore1 = 0;
          break;
        }

    }
  }

  // Heartbeat LED
  CR1_ulHeartbeatTimerNow = millis();
  if (CR1_ulHeartbeatTimerNow - CR1_ulHeartbeatTimerPrevious >= CR1_ciHeartbeatInterval)
  {
    CR1_ulHeartbeatTimerPrevious = CR1_ulHeartbeatTimerNow;
    btHeartbeat = !btHeartbeat;
    digitalWrite(ciHeartbeatLED, btHeartbeat);
    // Serial.println((vui32test2 - vui32test1)* 3 );
  }

}

//*******************************************************************************************************

void Roller(){
  if(rollerState == 0){
    rollerState = 1;
  }else{
    rollerState =0;
  }
}

//***************************************************************************************************************
