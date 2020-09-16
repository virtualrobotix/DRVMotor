#include <DRV8835MotorShield.h>

#define LED_PIN 13
#define VELMODE_PIN 2
#define RESET_PIN 4

#define CH3_IN_PIN 22
#define CH1_IN_PIN 23
#define VELMODE_IN_PIN 21
#define RESET_IN_PIN 20

#define CH3_FLAG 1
#define CH1_FLAG 2
#define VELMODE_FLAG 4
#define RESET_FLAG 8

#define CHANNEL_MIN 1100
#define CHANNEL_MAX 1900
#define CHANNEL_TRIM 1500
#define CHANNEL_DZ 30

volatile uint8_t bUpdateFlagsShared;

volatile uint16_t unCH3InShared;
volatile uint16_t unCH1InShared;
volatile uint16_t unVELMODEInShared;
volatile uint16_t unRESETInShared;

uint32_t ulCH3Start;
uint32_t ulCH1Start;
uint32_t ulVELMODEStart;
uint32_t ulRESETStart;

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
    if(unCH3InShared > CHANNEL_MAX)
    {
      unCH3InShared = CHANNEL_MAX;     
    }
    else if((unCH3InShared > (CHANNEL_TRIM - CHANNEL_DZ)) && (unCH3InShared < (CHANNEL_TRIM + CHANNEL_DZ)))
    {
      unCH3InShared = CHANNEL_TRIM;     
    }
    else if(unCH3InShared < CHANNEL_MIN)
    {
      unCH3InShared = CHANNEL_MIN;     
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
    if(unCH1InShared > CHANNEL_MAX)
    {
      unCH1InShared = CHANNEL_MAX;     
    }
    else if((unCH1InShared > (CHANNEL_TRIM - CHANNEL_DZ)) && (unCH1InShared < (CHANNEL_TRIM + CHANNEL_DZ)))
    {
      unCH1InShared = CHANNEL_TRIM;     
    }
    else if(unCH1InShared < CHANNEL_MIN)
    {
      unCH1InShared = CHANNEL_MIN;     
    }
    // use set the CH3 flag to indicate that a new CH3 signal has been received
    bUpdateFlagsShared |= CH1_FLAG;
  }
}

// simple interrupt service routine
void calcVELMODE()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(VELMODE_IN_PIN) == HIGH)
  {
    ulVELMODEStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unVELMODEInShared = (uint16_t)(micros() - ulVELMODEStart);
    if(unVELMODEInShared > CHANNEL_MAX)
    {
      unVELMODEInShared = CHANNEL_MAX;     
    }
    else if(unVELMODEInShared < CHANNEL_MIN)
    {
      unVELMODEInShared = CHANNEL_MIN;     
    }
    // use set the VELMODE flag to indicate that a new VELMODE signal has been received
    bUpdateFlagsShared |= VELMODE_FLAG;
  }
}

// simple interrupt service routine
void calcRESET()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(RESET_IN_PIN) == HIGH)
  {
    ulRESETStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unRESETInShared = (uint16_t)(micros() - ulRESETStart);
    if(unRESETInShared > CHANNEL_MAX)
    {
      unRESETInShared = CHANNEL_MAX;     
    }
    else if(unRESETInShared < CHANNEL_MIN)
    {
      unRESETInShared = CHANNEL_MIN;     
    }
    // use set the RESET flag to indicate that a new RESET signal has been received
    bUpdateFlagsShared |= RESET_FLAG;
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(115200);
 
  Serial.println("multiChannels");

  pinMode(VELMODE_PIN, OUTPUT);
  digitalWrite(VELMODE_PIN, LOW);

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);

  attachInterrupt(CH3_IN_PIN, calcCH3,CHANGE);
  attachInterrupt(CH1_IN_PIN, calcCH1,CHANGE);
  attachInterrupt(VELMODE_IN_PIN, calcVELMODE,CHANGE);
  attachInterrupt(RESET_IN_PIN, calcRESET,CHANGE);

  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipM1(true);
  //motors.flipM2(true);
}

void loop()
{
  static uint16_t unCH3In;
  static uint16_t unCH1In;
  static uint16_t unVELMODEIn;
  static uint16_t unRESETIn;

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
   
    if(bUpdateFlags & VELMODE_FLAG)
    {
      unVELMODEIn = unVELMODEInShared;
    }
   
    if(bUpdateFlags & RESET_FLAG)
    {
      unRESETIn = unRESETInShared;
    }
    
    bUpdateFlagsShared = 0;
   
    interrupts();
  }

  if(bUpdateFlags & CH3_FLAG)
  {
    digitalWrite(LED_PIN, HIGH);
    
    Motor1Speed = unCH3In - CHANNEL_TRIM;
    
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
    
    Motor2Speed = unCH1In - CHANNEL_TRIM;
    
    Serial.print("servoCH1: ");
    Serial.print(unCH1In);
    Serial.print(" - ");
    Serial.print(Motor2Speed);
    Serial.println("");
        
    motors.setM2Speed(Motor2Speed);
    delay(2);
    
    digitalWrite(LED_PIN, LOW);
  }
 
  if(bUpdateFlags & VELMODE_FLAG)
  {
    digitalWrite(LED_PIN, HIGH);
    
    Serial.print("servoVELMODE: ");
    Serial.print(unVELMODEIn);
    Serial.print(" - ");
        
    if (unVELMODEIn < CHANNEL_TRIM)
    {
      digitalWrite(VELMODE_PIN, LOW);
      Serial.print("Slow");
    }
    else
    {
      digitalWrite(VELMODE_PIN, HIGH);
      Serial.print("Fast");
    }
    Serial.println("");
    
    digitalWrite(LED_PIN, LOW);
  }
 
  if(bUpdateFlags & RESET_FLAG)
  {
    digitalWrite(LED_PIN, HIGH);
    
    Serial.print("servoRESET: ");
    Serial.print(unRESETIn);
    Serial.print(" - ");
        
    if (unRESETIn < CHANNEL_TRIM)
    {
      digitalWrite(RESET_PIN, LOW);
      Serial.print("Restore");
    }
    else
    {
      digitalWrite(RESET_PIN, HIGH);
      Serial.print("Reset");
    }
    Serial.println("");
    
    digitalWrite(LED_PIN, LOW);
  }
 
  bUpdateFlags = 0;
}
