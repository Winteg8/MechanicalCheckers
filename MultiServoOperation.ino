//
//  main.c
//  ArmTesting
//
//  Created by Maxim Anaya on 4/9/17.
//  Copyright © 2017 Maxim Anaya. All rights reserved.
//

#include <stdio.h>
#include "esp32-hal-ledc.h"
#include "Arduino.h"
#include "math.h"

// mode
const int pMode = 1;  // 0 for callibrating, 1 for running

// math
const double pie = 3.141592653589793;

// servo capabilities
const int period = 16;
const int minDelay0 = 2575;
const int maxDelay0 = 8425;
const int minDelay1 = 1600;
const int maxDelay1 = 7500;
const int minDelay2 = 1450;
const int maxDelay2 = 7550;
const int minDegree = 0;
const int maxDegree = 180;


// pin configuration
const int switchPin = 13;
const int EMPin = 22;
const int servoPin1 = 19;
const int servoPin2 = 2;
const int servoPin3 = 4;

// mechanical arm specifications (in cm and degrees)
const double B = 20.3200;
const double vClearance = 3; // height above the pickup/dropoff height of electromagnet





// mechanical arm functions
double armLength(double h)
{
    return (hypot(h, h))/2;
}
double height(double b) // B = board length
{
    double len = b*7/16;
    return hypot(len, len);
}
double horizontalDistance(double b, int squaresX, int squaresZ)
{
    float dx = (abs(squaresX) * b / 8) - (b / 16);
    float dz = (abs(squaresZ) * b / 8) - (b / 16);
    
    return (hypot(dx, dz));
}

double servoAngle0(int dx, int dz)
{
    float angle = atan((fabsf((float)dz) - 0.5)/(fabsf((float)dx) - 0.5)) * 180 / pie;

    /*Serial.print("dx = ");
    Serial.println(dx);
    Serial.print("dz = ");
    Serial.println(dz);
    Serial.print("dz/dx = ");
    Serial.println(dz/dx);
    Serial.println(fabsf((float)dz/(float)dx));
    Serial.print("Inter-quadrant angle = ");
    Serial.println(angle);*/
    if (dx > 0)
    {
      if (dz < 0)
        angle = 360 - angle;      // if not 4th quadrant (when x is positive), then 1st quadrant
    }   
    else if (dz > 0)
        angle = 180 - angle;      // 2nd quadrant
    else
        angle += 180;             // 3rd quadrant
    
    return map(angle, 0, 360, 180, 0); // such mapping is due to 2:1 inverse gearbox on the radial servo
}

double deg(double v)
{
    return (v * 180 / pie);
}

double m(double l, double h, double d)
{
    double answer = hypot(h,d)/2/l;
    if (answer > 1)
        return 1;
    else
        return answer;
}

double theta(double m)
{
    return 2*asin(m);
}

double zhe(double m)
{
    return acos(m);
}

double exx(double h, double d)
{
    return atan(h/d);
}

double phi(double l, double h, double d)
{
    return exx(h, d) - zhe(m(l, h, d));
}

double servoAngle1(double l, double h, double d)
{
    //printf("Rad1 = %f \n", phi(l, h, d));
    return deg(phi(l, h, d)) * 2; //map(deg(phi(l, h, d)), 0, 90, 0, 180); // mapping due to servo orientation (flipped 0 and 180)
}

double servoAngle2(double l, double h, double d)
{
    //printf("Rad2 = %f \n", theta(m(l, h, d)));
    return map((deg(theta(m(l, h, d))) - 60), 0, 120, 180, 0); // mapping due to 1.5 inverse gearbox on the servo
}


double doubleCheck(double l, double h, double d)
// returns horizontal distance (center to square) covered by the mechanical arm
{
    double d1 = l * cos(phi(l, h, d));
    double d2 = l * cos(theta(m(l, h, d)) - phi(l, h, d));
    double a2 = deg(theta(m(l, h, d) - phi(l, h, d)));
    
    printf("d1 = %f \n", d1);
    printf("d2 = %f \n", d2);
    printf("a2 = %f \n", a2);
    
    if (a2 >= 120)
    {
        printf("total distance = %f \n", d1+d2);
        return d1+d2;
    }
    else
    {
        printf("total distance = %f \n", d1-d2);
        return d1-d2;
    }
}



void moveComplex(float b, float l, float minD, float minR, float h, int ax, int az, int bx, int bz)
{
    // PROCEDURE:
    // calculate:
    // 1. How many points to calculate on the path from initial to final position
    //      - calculate minimum points to traverse distance (given a maximum distance between 2 points)
    //      - calculate minimum points to traverse angle (given a maximum angle between two angle values)
    //      - number of points for both angle and distance = greater of two minimums
    // 2. Where those poits are located
    // 3. Servo angles for each point
    // 4. Signal lengths mapped from servo angles
    // Output those signal lengths successively in a while loop

    Serial.print("B = ");
    Serial.print(b);
    Serial.print("      L = ");
    Serial.print(l);
    Serial.print("      h = ");
    Serial.print(h);

    
    float d1 = horizontalDistance(b, ax, az);
    float d2 = horizontalDistance(b, bx, bz);
    float a1 = servoAngle0(ax, az);
    float a2 = servoAngle0(bx, bz);
    Serial.print("d1 = ");
    Serial.println(d1);
    Serial.print("Servo1 at d1 = ");
    Serial.println(servoAngle1(l, h, d1));
    Serial.print("Servo2 at d1 = ");
    Serial.println(servoAngle2(l, h, d1));
    
    Serial.print("d2 = ");
    Serial.println(d2);
    Serial.print("Servo1 at d2 = ");
    Serial.println(servoAngle1(l, h, d2));
    Serial.print("Servo2 at d2 = ");
    Serial.println(servoAngle2(l, h, d2));
    
    Serial.print("servo0 1 = ");
    Serial.println(a1);
    Serial.print("servo0 2 = ");
    Serial.println(a2);
    
    // 1
    float minLinearPts = ceil(fabsf((d2 - d1)/minD));
    float minRadialPts = ceil(fabsf((a2 - a1)/minR));

    Serial.print("minLinearPts = fabsf(");
    Serial.print(d2);
    Serial.print(" - ");
    Serial.print(d1);
    Serial.print(") / ");
    Serial.print(minD);
    Serial.print(" = ");
    Serial.print(fabsf(d2-d1));
    Serial.print(" / ");
    Serial.print(minD);
    Serial.print(" = ");
    Serial.print((fabsf(d2-d1)) / minD);
    Serial.print(" == ");
    Serial.println(minLinearPts);
    Serial.print("minRadialPts = ");
    Serial.println(minRadialPts);
    
    if (minLinearPts > minRadialPts)
        minRadialPts = minLinearPts;
    else
        minLinearPts = minRadialPts;
        
    float intervalDist = (d2 - d1)/minLinearPts;
    float intervalAngl = (a2 - a1)/minRadialPts;

    Serial.println("Both are set to:");
    Serial.print("linear = ");
    Serial.println(minLinearPts);
    Serial.print("radial = ");
    Serial.println(minRadialPts);
    
    Serial.print("linearInterval = ");
    Serial.println(intervalDist);
    Serial.print("radialInterval = ");
    Serial.println(intervalAngl);
    
    // 2
    float linPts[(int)minLinearPts + 1];
    float radPts[(int)minRadialPts + 1];
    
    for (int i = 0; i <= (int)minLinearPts; i++)
    {
      linPts[i] = d1 + i * intervalDist;
      radPts[i] = a1 + i * intervalAngl;
    }
    
    // 3
    //float servo0[(int)minLinearPts];
    float servo1[(int)minLinearPts];
    float servo2[(int)minLinearPts];
    
    for (int i = 0; i <= (int)minLinearPts; i++)
    {
        // no need to convert angle to angle, only distance to angle
        servo1[i] = servoAngle1(l, h, linPts[i]);
        servo2[i] = servoAngle2(l, h, linPts[i]);

    }

    float value;
    Serial.println("MOVING...");
    Serial.println("---------------------");
    // 4
    for (int i = 0; i <= (int)minLinearPts; i++)
    {
        ledcWrite(1, map(radPts[i], minDegree, maxDegree, minDelay0, maxDelay0));
        ledcWrite(2, map(servo1[i], minDegree, maxDegree, minDelay1, maxDelay1));
        ledcWrite(3, map(servo2[i], minDegree, maxDegree, minDelay2, maxDelay2));
        Serial.print("servo0 = ");
        Serial.print(radPts[i]);
        Serial.print("              servo1 = ");
        Serial.print(servo1[i]);
        Serial.print("              servo2 = ");
        Serial.println(servo2[i]);

        delay(10);
    }
}
/*
void moveLinear(float B, float L, float minD, float h, int ax, int az, int bx, int bz)
{
    // PROCEDURE:
    // calculate:
    // 1. How many points to calculate on the path from initial to final position
    //      - calculate minimum points to traverse distance (given a maximum distance between 2 points)
    // 2. Where those poits are located
    // 3. Servo angles for each point
    // 4. Signal lengths mapped from servo angles
    // Output those signal lengths successively in a while loop
    // ----------------------------------------
    
    float d1 = horizontalDistance(B, ax, az);
    float d2 = horizontalDistance(B, bx, bz);
    
    // 1
    float minLinearPts = ceil(fabsf((d2 - d1)/minD));
    float intervalDist = (d2 - d1)/minLinearPts;
    
    // 2
    float points[(int)minLinearPts];
    if (d2 > d1)
        for (int i = 0; i < (int)minLinearPts; i++)
            points[i] = d1 + i * intervalDist;
    else
        for (int i = 0; i < (int)minLinearPts; i++)
            points[i] = d1 - i * intervalDist;
    
    // 3
    //float servo0[(int)minLinearPts];
    float servo1[(int)minLinearPts];
    float servo2[(int)minLinearPts];
    
    for (int i = 0; i < (int)minLinearPts; i++)
    {
        servo1[i] = servoAngle1(L, h, points[i]);
        servo2[i] = servoAngle2(L, h, points[i]);
    }
    
    // 4
    for (int i = 0; i < (int)minLinearPts; i++)
    {
        servo1[i] = map(servo1[i], 0, 180, 1000, 2000);
        servo2[i] = map(servo2[i], 0, 180, 1000, 2000);
    }
    
    for (int i = 0; i < (int)minLinearPts; i++)
    {
        ledcWrite(1, servo1[i]);
        ledcWrite(2, servo2[i]);
        
        delay(10);
    }
}*/

void moveVertically(float b, float l, float minD, float h1, float h2, int x, int z)
{
    float d = horizontalDistance(b, x, z);

    Serial.println("VERTICAL MOVE");
    Serial.print("d = ");
    Serial.println(d);
    Serial.print("h1 = ");
    Serial.println(h1);
    Serial.print("Servo1 at h1 = ");
    Serial.println(servoAngle1(l, h1, d));
    Serial.print("Servo2 at h1 = ");
    Serial.println(servoAngle2(l, h1, d));
    
    Serial.print("h2 = ");
    Serial.println(h2);
    Serial.print("Servo1 at h2 = ");
    Serial.println(servoAngle1(l, h2, d));
    Serial.print("Servo2 at h2 = ");
    Serial.println(servoAngle2(l, h2, d));
    Serial.println();
    
    // 1
    float minLinearPts = ceil(fabsf((h2 - h1)/minD));
    float intervalDist = (h2 - h1)/minLinearPts;
    
    // 2
    float points[(int)minLinearPts + 1];
      for (int i = 0; i <= (int)minLinearPts; i++)
            points[i] = h1 + i * intervalDist;
        
    // 3
    //float servo0[(int)minLinearPts];
    float servo1[(int)minLinearPts];
    float servo2[(int)minLinearPts];
    
    for (int i = 0; i <= (int)minLinearPts; i++)
    {
        servo1[i] = servoAngle1(l, points[i], d);
        servo2[i] = servoAngle2(l, points[i], d);
        /*Serial.print(i);
        Serial.print(". heght = ");
        Serial.print(points[i]);
        Serial.print("     a1 = ");
        Serial.print(servo1[i]);
        Serial.print("     a2 = ");
        Serial.println(servo2[i]);*/
        
    }
    
    // 4
    for (int i = 0; i <= (int)minLinearPts; i++)
    {
        ledcWrite(2, map(servo1[i], minDegree, maxDegree, minDelay1, maxDelay1));
        ledcWrite(3, map(servo2[i], minDegree, maxDegree, minDelay2, maxDelay2));
        Serial.print("servo1 = ");
        Serial.print(servo1[i]);
        Serial.print("              servo2 = ");
        Serial.println(servo2[i]);
        
        delay(20);
    }
}

void pickUp(float b, float l, float minD, float minR, float hHigh, float hLow, int ax, int az, int bx, int bz)
{
    // pre conditions: arm is at [ax, az] coords at hCarry height.
    // PROCEDURE:
    // 1. move linearly and/or radially
    // 2. call moveInterval to move vertically to height hReg
    // 3. toggle the electromagnet
    // 4. call moveInterval to move vertically to height hCarry
    
    
    moveComplex(b, l, minD, minR, hHigh, ax, az, bx, bz);
    delay(300);
    moveVertically(b, l, minD, hHigh, hLow, bx, bz);
    delay(150);
    pinMode(EMPin, HIGH); // turn electromagnet ON
    delay(150);
    moveVertically(b, l, minD, hLow, hHigh, bx, bz);
    delay(300);
}

void dropOff(float b, float l, float minD, float minR, float hHigh, float hLow, int ax, int az, int bx, int bz)
{
    // pre conditions: arm is at [ax, az] coords at hCarry height.
    // PROCEDURE:
    // 1. move linearly and/or radially
    // 2. call moveInterval to move vertically to height hReg
    // 3. toggle the electromagnet
    // 4. call moveInterval to move vertically to height hCarry
    
    
    moveComplex(b, l, minD, minR, hHigh, ax, az, bx, bz);
    delay(300);
    moveVertically(b, l, minD, hHigh, hLow, bx, bz);
    delay(150);
    pinMode(EMPin, LOW); // turn electromagnet OFF
    delay(150);
    moveVertically(b, l, minD, hLow, hHigh, bx, bz);
    delay(300);
}

void removePiece(float b, float l, float minD, float minR, float h, int dx, int dz)
{
    // same as moveComplex, but the destination is always off the board (where the eaten pieces are)
  // PROCEDURE:
    // calculate:
    // 1. How many points to calculate on the path from initial to final position
    //      - calculate minimum points to traverse distance (given a maximum distance between 2 points)
    //      - calculate minimum points to traverse angle (given a maximum angle between two angle values)
    //      - number of points for both angle and distance = greater of two minimums
    // 2. Where those poits are located
    // 3. Servo angles for each point
    // 4. Signal lengths mapped from servo angles
    // Output those signal lengths successively in a while loop
    
    float d1 = horizontalDistance(b, dx, dz);
    float d2 = 12.5;
    float a1 = servoAngle0(dx, dz);
    float a2 = 45;
    Serial.print("d1 = ");
    Serial.println(d1);
    Serial.print("Servo1 at d1 = ");
    Serial.println(servoAngle1(l, h, d1));
    Serial.print("Servo2 at d1 = ");
    Serial.println(servoAngle2(l, h, d1));
    
    Serial.print("d2 = ");
    Serial.println(d2);
    Serial.print("Servo1 at d2 = ");
    Serial.println(servoAngle1(l, h, d2));
    Serial.print("Servo2 at d2 = ");
    Serial.println(servoAngle2(l, h, d2));
    
    Serial.print("servo0 1 = ");
    Serial.println(a1);
    Serial.print("servo0 2 = ");
    Serial.println(a2);
    
    // 1
    float minLinearPts = ceil(fabsf((d2 - d1)/minD));
    float minRadialPts = ceil(fabsf((a2 - a1)/minR));

    Serial.print("minLinearPts = fabsf(");
    Serial.print(d2);
    Serial.print(" - ");
    Serial.print(d1);
    Serial.print(") / ");
    Serial.print(minD);
    Serial.print(" = ");
    Serial.print(fabsf(d2-d1));
    Serial.print(" / ");
    Serial.print(minD);
    Serial.print(" = ");
    Serial.print((fabsf(d2-d1)) / minD);
    Serial.print(" == ");
    Serial.println(minLinearPts);
    Serial.print("minRadialPts = ");
    Serial.println(minRadialPts);
    
    if (minLinearPts > minRadialPts)
        minRadialPts = minLinearPts;
    else
        minLinearPts = minRadialPts;
        
    float intervalDist = (d2 - d1)/minLinearPts;
    float intervalAngl = (a2 - a1)/minRadialPts;

    Serial.println("Both are set to:");
    Serial.print("linear = ");
    Serial.println(minLinearPts);
    Serial.print("radial = ");
    Serial.println(minRadialPts);
    
    Serial.print("linearInterval = ");
    Serial.println(intervalDist);
    Serial.print("radialInterval = ");
    Serial.println(intervalAngl);
    
    // 2
    float linPts[(int)minLinearPts + 1];
    float radPts[(int)minRadialPts + 1];
    
    for (int i = 0; i <= (int)minLinearPts; i++)
    {
      linPts[i] = d1 + i * intervalDist;
      radPts[i] = a1 + i * intervalAngl;
    }
    
    // 3
    //float servo0[(int)minLinearPts];
    float servo1[(int)minLinearPts];
    float servo2[(int)minLinearPts];
    
    for (int i = 0; i <= (int)minLinearPts; i++)
    {
        // no need to convert angle to angle, only distance to angle
        servo1[i] = servoAngle1(l, h, linPts[i]);
        servo2[i] = servoAngle2(l, h, linPts[i]);
        /*Serial.print(i);
        Serial.print(". distance = ");
        Serial.print(linPts[i]);
        Serial.print("     a1 = ");
        Serial.print(servo1[i]);
        Serial.print("     a2 = ");
        Serial.println(servo2[i]);*/
    }

    float value;
    Serial.println("MOVING...");
    Serial.println("---------------------");
    // 4
    for (int i = 0; i <= (int)minLinearPts; i++)
    {
        ledcWrite(1, map(radPts[i], minDegree, maxDegree, minDelay0, maxDelay0));
        ledcWrite(2, map(servo1[i], minDegree, maxDegree, minDelay1, maxDelay1));
        ledcWrite(3, map(servo2[i], minDegree, maxDegree, minDelay2, maxDelay2));
        delay(10);
    }

    pinMode(EMPin, LOW); // turn off electromagnet
}

int checkerX(char x)
{
  switch (x)
  {
    case 'a':
      return -4;
    case 'b':
      return -3;
    case 'c':
      return -2;
    case 'd':
      return -1;
    case 'e':
      return 1;
    case 'f':
      return 2;
    case 'g':
      return 3;
    case 'h':
      return 4;
  }
}

int checkerZ(int z)
{
  if (z > 4)
    return (z - 4);
  else
    return (z - 5);
}


void turn(float a1, float a2, float maxR, int p, int servoNum)
{
  // function meant for servo callibration
  
  int numPts = ceil(fabsf(a2 - a1) / maxR);
  Serial.print("numPts = ");
  Serial.println(numPts);
  float angleInterval = (a2 - a1) / numPts;
  Serial.print("Interval angle = ");
  Serial.println(angleInterval);
  Serial.print("Distance to cover = ");
  Serial.println(numPts*angleInterval);
  Serial.print("Starting at ");
  Serial.println(a1);
  //Serial.print(", ending at 
  float value = 0;
  float minD = 0;
  float maxD = 0;

  switch (servoNum)
  {
    case 0:
    {
      minD = minDelay0;
      maxD = maxDelay0;
    }
    case 1:
    {
      minD = minDelay1;
      maxD = maxDelay1;
    }
    case 2:
    {
      minD = minDelay2;
      maxD = maxDelay2;
    }
  }

  
  for (int i = 0; i <= numPts; i++)
  {
    value = a1 + i * angleInterval;
    Serial.print("valPos = ");
    Serial.println(value);
    ledcWrite((servoNum + 1), map(value, minDegree, maxDegree, minD, maxD));
    delay(p);
  }

  delay(1000);
}






void setup()
{
    Serial.begin(9600);
    // mechanical arm calculations
    const double hReg = height(B);
    const double hCarry = hReg - vClearance;
    const double L = armLength(hReg);
    int dx = 4;
    int dz = 4;
    
    double d = horizontalDistance(B, dx, dz);
    double dmax = horizontalDistance(B, 4, 4);
    
    int servo1_1 = 0;
    int servo1_2 = servoAngle1(L, hReg, d);
    int servo2_1 = 0;
    int servo2_2 = servoAngle2(L, hReg, d);
    
    
    printf("\nBoard length = ");
    printf("%f \n", B);
    printf("Maximum distance d = ");
    printf("%f \n", dmax);
    printf("L = ");
    printf("%f \n", L);
    printf("Custom distance = ");
    printf("%f \n", d);
    printf("\n   Piece pickup/drop values: \n");
    printf("h = ");
    printf("%f \n", hReg);
    //printf("Hypotenuse = ");
    //printf("%f \n", hyp);
    
    
    servo1_2 = servoAngle1(L, hReg, d);
    printf("Servo 1 angle = ");
    printf("%d \n", servo1_2);
    servo2_2 = servoAngle2(L, hReg, d);
    printf("Servo 2 angle = ");
    printf("%d \n", servo2_2);
    doubleCheck(L, hReg, d);
    
    printf("\n   Piece carrying values: \n");
    printf("h = ");
    printf("%f \n", hCarry);
    servo1_1 = servoAngle1(L, hCarry, d);
    printf("Servo 1 angle = ");
    printf("%d \n", servo1_1);
    servo2_1 = servoAngle2(L, hCarry, d);
    printf("Servo 2 angle = ");
    printf("%d \n", servo2_1);
    doubleCheck(L, hCarry, d);


    pinMode(switchPin, INPUT);
    pinMode(EMPin, OUTPUT);
    ledcSetup(1, 50, period); // channel 1, 50 Hz, 16-bit width
    ledcAttachPin(servoPin1, 1);   // GPIO 19 assigned to channel 1
    ledcSetup(2, 50, 16); // channel 2, 50 Hz, 16-bit width
    ledcAttachPin(servoPin2, 2);   // GPIO 2 assigned to channel 2
    ledcSetup(3, 50, period); // channel 3, 50 Hz, 16-bit width
    ledcAttachPin(servoPin3, 3);   // GPIO 4 assigned to channel 3

    Serial.println();
    Serial.println("Radial angle check");
    Serial.println("-------------------------");
    char letter = 'a';
    int number = 1;
    Serial.println(servoAngle0(checkerX(letter), checkerZ(number)));
    
}

void loop()
{
  if (digitalRead(switchPin) == 1)
  {
    const double hReg = height(B);
    const double hCarry = hReg - vClearance;
    const double L = armLength(hReg);
    const float minLinear = 0.25; // [cm]
    const float minRadial = 2; // [degrees]

   

    if (digitalRead(switchPin) == 1)
    {
      if (pMode == 1)
      {
        pickUp(B, L, minLinear, minRadial, hCarry, hReg, -4, -4, 1, 1);
        //dropOff(B, L, minLinear, minRadial, hCarry, hReg, 1, 1, -4, -4);
        dropOff(B, L, minLinear, minRadial, hCarry, hReg, 1, 1, -3, 4);
        pickUp(B, L, minLinear, minRadial, hCarry, hReg, -3, 4, -2, 1);
        dropOff(B, L, minLinear, minRadial, hCarry, hReg, -2, 1, -4, -4);
      }
      else if (pMode == 0)
      {
        turn(0, 180, 1, 20, 0);
        if (digitalRead(switchPin) == 1)
          turn(180, 0, 1, 20, 0);
      }
    }  
  }
}

