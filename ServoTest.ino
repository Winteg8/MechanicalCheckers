#include "Arduino.h"

const int minDelay = 500;
const int maxDelay = 2300;
const int minDegree = 0;
const int maxDegree = 180;
const int switchPin = 13;
const int servoPin = 2;


const int degree1 = 0;
const int degree2 = 180;
int delay1 = 0;
int delay2 = 0;
void setup() 
{
  pinMode(servoPin, OUTPUT);
  pinMode(switchPin, INPUT);
  Serial.begin(9600);
}

void loop()
{
  delay1 = map(degree1, minDegree, maxDegree, minDelay, maxDelay);
  delay2 = map(degree2, minDegree, maxDegree, minDelay, maxDelay);
  Serial.println(delay1);
  Serial.println(delay2);

  // find the mean of the two delays
  float mean = (delay1 + delay2) / 2;
  float acceleration = 0.1;
  float jerk = 0.02;
  Serial.print ("acceleration = ");
  Serial.println(acceleration);
  float sum = 0;
  
  int switchVal = digitalRead(switchPin);
  Serial.println("Moving up...");
  for (float i = delay1; i < delay2; i+=sum)
  {
    if (i <= mean)
    {
      sum += acceleration;
      acceleration += jerk; 
    }
    
    if (switchVal == 1)
    {
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(i);
      digitalWrite(servoPin, LOW);
    }

    if (i > mean)
    {
      acceleration -= jerk;
      sum -= acceleration;
      
    }

    
    Serial.print(sum);
    Serial.print("   ");
    Serial.println(i);

    delay(3);
  }
  delay(1000);


  switchVal = digitalRead(switchPin);
  Serial.println("Moving down...");
  /*
  for (int i = delay2; i > delay1; i-=2)
  {
    if (switchVal == 1)
    {
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(i);
      digitalWrite(servoPin, LOW);
    }
    
    delay(3);
  }*/
  sum = 0;
  for (float i = delay2; i > delay1; i-=sum)
  {
    if (i >= mean)
    {
      sum += acceleration;
      acceleration += jerk; 
    }

    if (switchVal == 1)
    {
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(i);
      digitalWrite(servoPin, LOW);
    }

    if (i < mean)
    {
      acceleration -= jerk;
      sum -= acceleration;
      
    }
    
    Serial.print(sum);
    Serial.print("   ");
    Serial.println(i);

    delay(3);
  }
  delay(1000);
    

  
  
}
