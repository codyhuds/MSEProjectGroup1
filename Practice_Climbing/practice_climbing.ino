const int windupwheelA = 2;
const int windupwheelB = 5;
const int servoPin = 15;
const int servoChannel = 7;

int currentTimer;
int previousTimer = 0;
int motorindex = 0;
int servoPos;
int Position;

long degreesToDutyCycle(int deg){
  const long minDutyCycle = 1675;
  const long maxDutyCycle = 8050;

long dutyCycle = map(deg, 0, 180, minDutyCycle, maxDutyCycle);

  return dutyCycle;
}


void setup() {
   ledcAttachPin(windupwheelA, 5); // assign Motors pins to channels
   ledcAttachPin(windupwheelB, 6);
   ledcSetup(5, 20000, 8); // 20mS PWM, 8-bit resolution
   ledcSetup(6, 20000, 8);
   
   ledcAttachPin(servoPin, servoChannel);
   ledcSetup(servoChannel, 50, 16);
   
   ledcWrite(servoChannel, degreesToDutyCycle(180));
}

void loop() {
  currentTimer = millis();
  if(currentTimer - previousTimer >= 2000)
  {
    previousTimer = millis();
    switch(motorindex){
      case 0: //wait for cody
      {
        motorindex = 1;
        break;
      }
      case 1: //drop wheel
      {
        //servo position changes
        Position = 1;
        servoPos = map(Position, 0, 2, 0, 180);
        ledcWrite(servoChannel, degreesToDutyCycle(servoPos));
        motorindex = 2;
        break;
      }
      case 2: //clamp rope
      {
        ledcWrite(5, 255);
        ledcWrite(6, 0);
        motorindex = 3;
        break;
      }
      case 3: //pause
      {
        ledcWrite(5, 0);
        ledcWrite(6, 0);
        motorindex = 4;
        break;
      }
      case 4: //rotate wheel down
      {
        //servo position changes
        ledcWrite(servoChannel, degreesToDutyCycle(15));
        motorindex = 5;
        break;
      }
      case 5: //climb rest of the way
      {
        ledcWrite(5, 255);
        ledcWrite(6, 0);
        motorindex = 6;
        break;
      }
    }
  }
}
