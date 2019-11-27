#include <DRV8835MotorShield.h>

#define LED_PIN 13
#define MODE_PIN 7

#define CH3_IN_PIN 22
#define CH1_IN_PIN 23

#define CH3_FLAG 1
#define CH1_FLAG 2

#define CHANNEL_TRIM 1500
#define CHANNEL_DZ 50

volatile uint8_t bUpdateFlagsShared;

volatile uint16_t unCH3InShared;
volatile uint16_t unCH1InShared;

uint32_t ulCH3Start;
uint32_t ulCH1Start;

DRV8835MotorShield motors;

// simple interrupt service routine
void calcCH3()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(CH3_IN_PIN) == HIGH)
  {
    ulCH3Start = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unCH3InShared = (uint16_t)(micros() - ulCH3Start);
    if(unCH3InShared > 1900)
    {
      unCH3InShared = 1900;     
    }
    else if(unCH3InShared < 1100)
    {
      unCH3InShared = 1100;     
    }
    // use set the CH3 flag to indicate that a new CH3 signal has been received
    bUpdateFlagsShared |= CH3_FLAG;
  }
}

// simple interrupt service routine
void calcCH1()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(CH1_IN_PIN) == HIGH)
  {
    ulCH1Start = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unCH1InShared = (uint16_t)(micros() - ulCH1Start);
    if(unCH1InShared > 1900)
    {
      unCH1InShared = 1900;     
    }
    else if(unCH1InShared < 1100)
    {
      unCH1InShared = 1100;     
    }
    // use set the CH3 flag to indicate that a new CH3 signal has been received
    bUpdateFlagsShared |= CH1_FLAG;
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(115200);
 
  Serial.println("multiChannels");

  pinMode(MODE_PIN, OUTPUT);
  digitalWrite(MODE_PIN, HIGH);

  attachInterrupt(CH3_IN_PIN, calcCH3,CHANGE);
  attachInterrupt(CH1_IN_PIN, calcCH1,CHANGE);

  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipM1(true);
  //motors.flipM2(true);
}

void loop()
{
  static uint16_t unCH3In;
  static uint16_t unCH1In;

  static int Motor1Speed;
  static int Motor2Speed;

  static uint8_t bUpdateFlags;

  if(bUpdateFlagsShared)
  {
    noInterrupts();

    bUpdateFlags = bUpdateFlagsShared;
   
    if(bUpdateFlags & CH3_FLAG)
    {
      unCH3In = unCH3InShared;
    }
   
    if(bUpdateFlags & CH1_FLAG)
    {
      unCH1In = unCH1InShared;
    }
   
    bUpdateFlagsShared = 0;
   
    interrupts();
  }

  if(bUpdateFlags & CH3_FLAG)
  {
    digitalWrite(LED_PIN, HIGH);
    
    Motor1Speed = unCH3In - 1500;
    
    Serial.print("servoCH3: ");
    Serial.print(unCH3In);
    Serial.print(" - ");
    Serial.print(Motor1Speed);
    Serial.println("");
        
    motors.setM1Speed(Motor1Speed);
    delay(2);
    
    digitalWrite(LED_PIN, LOW);
  }
 
  if(bUpdateFlags & CH1_FLAG)
  {
    digitalWrite(LED_PIN, HIGH);
    
    Motor2Speed = unCH1In - 1500;
    
    Serial.print("servoCH1: ");
    Serial.print(unCH1In);
    Serial.print(" - ");
    Serial.print(Motor2Speed);
    Serial.println("");
        
    motors.setM2Speed(Motor2Speed);
    delay(2);
    
    digitalWrite(LED_PIN, LOW);
  }
 
  bUpdateFlags = 0;
}
