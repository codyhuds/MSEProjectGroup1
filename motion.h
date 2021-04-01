/*
 Western Engineering base code
2020 05 13 E J Porter

 Dual Motor driver controller implement
  
 */

 #ifndef MOTION_H
  #define MOTION_H 1

//---------------------------------------------------------------------------

#define DEBUGPRINT 1
#define ACCELERATIONRATE 1;



unsigned char ucMotion_Direction;
unsigned char ucMotion_Speed;

const uint8_t cui8StartingSpeed = 140;

uint8_t ui8LeftWorkingSpeed = cui8StartingSpeed;
uint8_t ui8RightWorkingSpeed = cui8StartingSpeed;

unsigned char ucMotorState = 0;

double dManualSpeed;
double dForwardSpeed;
double dReverseSpeed;
double dLeftSpeed;
double dRightSpeed;

void setupMotion (void)
{
	
  dManualSpeed = 0;
  dForwardSpeed = 250;  // max 255; min ~150 before motor stall
  dReverseSpeed = 250;
  dLeftSpeed = 170;
  dRightSpeed = 170;
  
  //setup PWM for motors
  ledcAttachPin(ciMotorLeftA, 1); // assign Motors pins to channels
  ledcAttachPin(ciMotorLeftB, 2);
  ledcAttachPin(ciMotorRightA, 3);
  ledcAttachPin(ciMotorRightB, 4);

  // Initialize channels 
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(1, 20000, 8); // 20mS PWM, 8-bit resolution
  ledcSetup(2, 20000, 8);
  ledcSetup(3, 20000, 8);
  ledcSetup(4, 20000, 8);
 	
   ucMotion_Direction = 0;
   ucMotion_Speed = 0;
}


void ResetSpeeds()
{
  ui8LeftWorkingSpeed = cui8StartingSpeed;
  ui8RightWorkingSpeed = cui8StartingSpeed;
}

void MoveTo(uint8_t ui8Direction, uint8_t ui8LeftSpeed, uint8_t ui8RightSpeed)
{
    int  iPrintOnce;
      
   
     switch(ui8Direction)
      {
      
      
        //forward
        case 1:
        {
            
          if(ui8LeftWorkingSpeed >= ui8LeftSpeed)
          {
            ui8LeftWorkingSpeed = ui8LeftSpeed;
          }
          else
          {
          ui8LeftWorkingSpeed = ui8LeftWorkingSpeed + ACCELERATIONRATE;
          }
          if(ui8RightWorkingSpeed >= ui8RightSpeed)
          {
            ui8RightWorkingSpeed = ui8RightSpeed;
          }
          else
          {
            ui8RightWorkingSpeed = ui8RightWorkingSpeed + ACCELERATIONRATE;
          }
          
          ledcWrite(2,0);
          ledcWrite(1,ui8LeftWorkingSpeed);
          ledcWrite(4,0);
          ledcWrite(3,ui8RightWorkingSpeed);
          
          break;
        }
        //Left
        case 2:
        {
          if(ui8LeftWorkingSpeed >= ui8LeftSpeed)
          {
            ui8LeftWorkingSpeed = ui8LeftSpeed;
          }
          else
          {
          ui8LeftWorkingSpeed = ui8LeftWorkingSpeed + ACCELERATIONRATE;
          }
          if(ui8RightWorkingSpeed >= ui8RightSpeed)
          {
            ui8RightWorkingSpeed = ui8RightSpeed;
          }
          else
          {
            ui8RightWorkingSpeed = ui8RightWorkingSpeed + ACCELERATIONRATE;
          }
         
          ledcWrite(1,0);
          ledcWrite(2,ui8LeftWorkingSpeed*2);
          ledcWrite(4,0);
          ledcWrite(3,ui8RightWorkingSpeed);
        
          break;
        }
        //Right
        case 3:
        {
          if(ui8LeftWorkingSpeed >= ui8LeftSpeed)
          {
            ui8LeftWorkingSpeed = ui8LeftSpeed;
          }
          else
          {
          ui8LeftWorkingSpeed = ui8LeftWorkingSpeed + ACCELERATIONRATE;
          }
          if(ui8RightWorkingSpeed >= ui8RightSpeed)
          {
            ui8RightWorkingSpeed = ui8RightSpeed;
          }
          else
          {
            ui8RightWorkingSpeed = ui8RightWorkingSpeed + ACCELERATIONRATE;
          }
         
          ledcWrite(2,0);
          ledcWrite(1,ui8LeftWorkingSpeed);
          ledcWrite(3,0);
          ledcWrite(4,ui8RightWorkingSpeed);
       
          break;
        }
        //Reverse
        case 4:
        {
             
          if(ui8LeftWorkingSpeed >= ui8LeftSpeed)
          {
            ui8LeftWorkingSpeed = ui8LeftSpeed;
          }
          else
          {
          ui8LeftWorkingSpeed = ui8LeftWorkingSpeed + ACCELERATIONRATE;
          }
          if(ui8RightWorkingSpeed >= ui8RightSpeed)
          {
            ui8RightWorkingSpeed = ui8RightSpeed;
          }
          else
          {
            ui8RightWorkingSpeed = ui8RightWorkingSpeed + ACCELERATIONRATE;
          }
         
          ledcWrite(1,0);
          ledcWrite(2,ui8LeftWorkingSpeed);
          ledcWrite(3,0);
          ledcWrite(4,ui8RightWorkingSpeed);
       
          break;
        }
     
        
      }
 }


#endif
