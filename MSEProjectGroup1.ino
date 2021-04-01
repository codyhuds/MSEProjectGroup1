
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
const int servoPin = 15;
const int servoChannel = 7;

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

//volatile int32_t ENC_vi32LeftOdometer;
//volatile int32_t ENC_vi32RightOdometer;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = 0;

int servoPos;
int Position;

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;
unsigned long previousMillis;
unsigned long currentMillis;

boolean btHeartbeat = true;
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;

const int CR1_ciMainTimer =  500;         // set time interval between subcases in "Case 0" which sets up the motor parameters
const int CR1_ciHeartbeatInterval = 500;
const int CR1_ciMotorRunTime = 2000;
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;
const long totalMillis = 200;

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

long degreesToDutyCycle(int deg){
  const long minDutyCycle = 1675;
  const long maxDutyCycle = 8050;

long dutyCycle = map(deg, 0, 180, minDutyCycle, maxDutyCycle);

  return dutyCycle;
}

void setup(){
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

   SmartLEDs.begin();
   SmartLEDs.clear();
   SmartLEDs.show();

   ledcAttachPin(servoPin, servoChannel);
   ledcSetup(servoChannel, 50, 16);

   ucMotorStateIndex = 0;
}

void loop(){
   WSVR_BreakPoint(1);

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
       //Serial.println(btRun); // uncomment to output whether btRun is true or false
       
       // if stopping, reset motor states and stop motors
       if(!btRun)
       {
          ucMotorStateIndex = 0; 
          ucMotorState = 0;
          MoveTo(0, 0, 0);
       }
     }
    }
  }
  iLastButtonState = iButtonValue; //store button state

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
  
 if(CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer){
  
  WDT_ResetCore1(); 
  WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;
   
  CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;

  switch(CR1_ucMainTimerCaseCore1)  //full switch run through is 1mS 
  {

    case 0:
    {
      if(btRun){
       CR1_ulMotorTimerNow = millis();
       if(CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime){
         CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
         switch(ucMotorStateIndex){
          case 0: //wait for Cody
          {
            ucMotorStateIndex = 1;
            break;
          }
          case 1: //go forward
          {
            ENC_SetDistance(280, 300);
            ucMotorState = 1;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 202;
            ucMotorStateIndex = 2;
            break;
          }
          case 2: //turn left
          {
            ENC_SetDistance(69, 38);
            ucMotorState = 2;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 201;
            ucMotorStateIndex = 3;
            break;
          }
          case 3: //go forward again
          {
            ENC_SetDistance(195, 240);
            ucMotorState = 1;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 202;
            ucMotorStateIndex = 4;
            break;
          }
          case 4: //turn right
          {
            ENC_SetDistance(-14, 41);
            ucMotorState = 3;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 201;
            ucMotorStateIndex = 5;
            break;
          }
          case 5: //go forward again
          {
            ENC_SetDistance(315, 300);
            ucMotorState = 1;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 202;
            ucMotorStateIndex = 6;
            break;
          }
          case 6: //turn right again
          {
            ENC_SetDistance(-16, 41);
            ucMotorState = 3;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 201;
            ucMotorStateIndex = 7;
            break;
          }
          case 7: //go forward again
          {
            ENC_SetDistance(470, 300);
            ucMotorState = 1;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 202;
            ucMotorStateIndex = 8;
            break;
          }
          case 8: //look for beacon
          {
            ENC_SetDistance(-5, 41);
            ucMotorState = 3;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 201;
            break;
          }
          case 9: //go to beacon
          {
            ENC_SetDistance(200, 200);
            ucMotorState = 1;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 202;
            break;
          }
          case 10: //go backwards
          {
            ENC_SetDistance(80, 20);
            ucMotorState = 4;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 199;
            ucMotorStateIndex = 11;
            break;
          }
          case 11: //celebratory circle
          {
            ENC_SetDistance(200, 200);
            ucMotorState = 2;
            CR1_ui8LeftWheelSpeed = 255;
            CR1_ui8RightWheelSpeed = 200;
            ucMotorStateIndex = 12;
            break;
          }
          case 12: //raise flag
          {
            Position = 1;
            servoPos = map(Position, 0, 2, 0, 180);
            ledcWrite(servoChannel, degreesToDutyCycle(servoPos));
            ucMotorStateIndex = 13;
            Serial.println(servoPos);
            break;
          }
          case 13: //lower flag
          {
            Position = 0;
            servoPos = map(Position, 0, 2, 0, 180);
            ledcWrite(servoChannel, degreesToDutyCycle(servoPos)); 
            ucMotorStateIndex = 14;
            Serial.println(servoPos);
            break;
          }
         }
       }
  }
    CR1_ucMainTimerCaseCore1 = 1;

    break;
    }

    case 1:
    {
      if(CR1_ui8IRDatum == 0x55 && ucMotorStateIndex == 8){
              ledcWrite(2,255);
              ledcWrite(1,255);
              ledcWrite(4,255);
              ledcWrite(3,255);
              ucMotorStateIndex = 9;
            }
      //if(CR1_ui8IRDatum == 0x41 && ucMotorStateIndex == 9){
              //ledcWrite(2,255);
              //ledcWrite(1,255);
              //ledcWrite(4,255);
              //ledcWrite(3,255);
              //ucMotorStateIndex = 10;
            //}

         
        if(!digitalRead(ciLimitSwitch))
        {
          ledcWrite(2,255);
          ledcWrite(1,255);
          ledcWrite(4,255);
          ledcWrite(3,255);
          ucMotorStateIndex = 10;
          Serial.println(digitalRead(ciLimitSwitch));
        }
            
      CR1_ucMainTimerCaseCore1 = 2;
      break;
    }
    case 2:
    {
      if(ENC_ISMotorRunning())
      {
        MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed,CR1_ui8RightWheelSpeed);
      }
      
      CR1_ucMainTimerCaseCore1 = 3;
      
      break;
    }

    case 3:
    {
      if (CR1_ui8IRDatum == 0x55) {                // if proper character is seen
         SmartLEDs.setPixelColor(0,0,25,0);         // make LED1 green with 10% intensity
         //Serial.println("Green");
       }
       else if (CR1_ui8IRDatum == 0x41) {           // if "hit" character is seen
         SmartLEDs.setPixelColor(0,25,0,25);        // make LED1 purple with 10% intensity
         //Serial.println("Purple");
       }
       else {                                       // otherwise
         SmartLEDs.setPixelColor(0,25,0,0);         // make LED1 red with 10% intensity
         //Serial.println("Red");
       }
       SmartLEDs.show();                            // send updated colour to LEDs
          
      CR1_ucMainTimerCaseCore1 = 0;
      break;
    }
 }

  // Heartbeat LED
 CR1_ulHeartbeatTimerNow = millis();
 if(CR1_ulHeartbeatTimerNow - CR1_ulHeartbeatTimerPrevious >= CR1_ciHeartbeatInterval)
 {
    CR1_ulHeartbeatTimerPrevious = CR1_ulHeartbeatTimerNow;
    btHeartbeat = !btHeartbeat;
    digitalWrite(ciHeartbeatLED, btHeartbeat);
   // Serial.println((vui32test2 - vui32test1)* 3 );
 }

}
}
