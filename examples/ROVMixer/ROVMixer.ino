#include <DRV8835MotorShield.h>

#define LED_PIN 13
#define MODE_PIN 6

#define THROTTLE_IN_PIN 22
#define STEERING_IN_PIN 23

#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

#define CHANNEL_TRIM 1500
#define CHANNEL_DZ 50

volatile uint8_t bUpdateFlagsShared;

volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;

uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

DRV8835MotorShield motors;

// simple interrupt service routine
void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    if(unThrottleInShared > 1900)
    {
      unThrottleInShared = 1900;     
    }
    else if(unThrottleInShared < 1100)
    {
      unThrottleInShared = 1100;     
    }
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

// simple interrupt service routine
void calcSteering()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    if(unSteeringInShared > 1900)
    {
      unSteeringInShared = 1900;     
    }
    else if(unSteeringInShared < 1100)
    {
      unSteeringInShared = 1100;     
    }
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(115200);
 
  Serial.println("multiChannels");

  pinMode(MODE_PIN, OUTPUT);
  digitalWrite(MODE_PIN, HIGH);

  attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  attachInterrupt(STEERING_IN_PIN, calcSteering,CHANGE);

  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipM1(true);
  //motors.flipM2(true);
}

void loop()
{
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;

  static int Motor1Speed;
  static int Motor2Speed;

  static uint8_t bUpdateFlags;

  if(bUpdateFlagsShared)
  {
    noInterrupts();

    bUpdateFlags = bUpdateFlagsShared;
   
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
   
    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
   
    bUpdateFlagsShared = 0;
   
    interrupts();
  }

  if(bUpdateFlags & THROTTLE_FLAG)
  {
    digitalWrite(LED_PIN, HIGH);
    
    Motor1Speed = unThrottleIn - 1500;
    
    Serial.print("servoThrottle: ");
    Serial.print(unThrottleIn);
    Serial.print(" - ");
    Serial.print(Motor1Speed);
    Serial.println("");
        
    motors.setM1Speed(Motor1Speed);
    delay(2);
    
    digitalWrite(LED_PIN, LOW);
  }
 
  if(bUpdateFlags & STEERING_FLAG)
  {
    digitalWrite(LED_PIN, HIGH);
    
    Motor2Speed = unSteeringIn - 1500;
    
    Serial.print("servoSteering: ");
    Serial.print(unSteeringIn);
    Serial.print(" - ");
    Serial.print(Motor2Speed);
    Serial.println("");
        
    motors.setM2Speed(Motor2Speed);
    delay(2);
    
    digitalWrite(LED_PIN, LOW);
  }
 
  bUpdateFlags = 0;
}
