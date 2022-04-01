#include <avr/pgmspace.h>
#define MOVE_POINTUPTHENDOWN_SIZE 49
#include "Servotor32.h"
//#include "UltrasonicClass.h"
Servotor32 hexy;

const uint8_t rightFootServo = 26;
const uint8_t rightKneeServo = 25;
const uint8_t rightHipServo = 24;
const float rightHipOffset = 30;

const uint8_t leftFootServo = 5;
const uint8_t leftKneeServo = 6;
const uint8_t leftHipServo = 7;
const float leftHipOffset = -30;

const uint8_t upPosition = 209;
const uint8_t downPosition = 74;

const float timeForFullScan = 10;
const float deltaUpdate = 0.2;
const float ignoreRange = 100;
const float rotateThresh = 45;

const uint8_t lookLeftPos = 50;
const uint8_t lookRightPos = 250;
const int lookLeftAngle = 90;
const int lookRightAngle = -90;

const uint8_t headServo = 31;

// IK constants
const float ARM_SIDE_OFFSET = 0.067;
const float ARM_FORWARD_OFFSET = 0;
const float SENSOR_HEIGHT = 0.065;
const float SENSOR_REDUCTION = 0.010;
const float WRIST_REDUCTION = 0.020;
const float L1 = 0.0275;
const float L2 = 0.050;

void setup() {
  // put your setup code here, to run once:
  hexy.begin();


  uint8_t pwmVal = (uint8_t)map(0,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
  hexy.changeServo(rightHipServo, pwmVal*10);  
  hexy.changeServo(rightKneeServo, pwmVal*10);
  hexy.changeServo(rightFootServo, pwmVal*10);
  // left leg
  hexy.changeServo(leftFootServo, pwmVal*10);
  hexy.changeServo(leftHipServo, pwmVal*10);
  hexy.changeServo(leftKneeServo, pwmVal*10);
//  hexy.delay_ms(1000000);
}

// extra trig functions
float cosd(float deg)
{
  return cos(deg*PI/180);
}
float acosd(float val)
{
  return acos(val)*180/PI;
}
float sind(float deg)
{
  return sin(deg*PI/180);
}
float asind(float val)
{
  return asin(val)*180/PI;
}
float atand(float val)
{
  return atan(val)*180/PI;
}

void HighFive(float range, float heading)
{
  // convert polar to cartesian, with origin centered at head rotation axis
  range = range + SENSOR_REDUCTION - WRIST_REDUCTION;
  float X = range*cosd(heading);
  float Y = range*sind(heading);

  // translate coordinate system to be centered on a hip.
  float x = X - ARM_FORWARD_OFFSET;
  float y = Y < 0 ? Y+ARM_SIDE_OFFSET:Y-ARM_SIDE_OFFSET;
  
  // calculate IK
  float r = sqrt(sq(x)+sq(y));
  float A = sqrt(sq(r) + sq(SENSOR_HEIGHT));
  float theta2 = -atand(SENSOR_HEIGHT/(r-L1));
  theta2 = constrain(theta2,-90,90);
  //theta2 = r-L1 <0 ? 180 - theta2 : theta2;
  float theta1 = atand(y/x);
  theta1 = constrain(theta1,-90,90);
  //theta1 = (x <0 ? 180 - theta1 : theta1);
  //Serial.println("theta1 = " + String(theta1));
  //if (theta1 > 180)
  //  theta1 -= 360;
  //else if (theta1 < -180)
  //  theta1 += 360;


  Serial.println("Heading = " + String(heading));
  Serial.println("Range = " + String(range));
  Serial.println("X = " + String(X));
  Serial.println("Y = " + String(Y));
  Serial.println("x = " + String(x));
  Serial.println("y = " + String(y));
  Serial.println("theta1 = " + String(theta1));
  Serial.println("theta2 = " + String(theta2));
  Serial.println("________");

  // perform IK
  if (heading <= 0)
  {
    // right leg
    uint8_t pwmVal = (uint8_t)map(90,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(rightFootServo,pwmVal*10);
    hexy.delay_ms(500);
    
    pwmVal = (uint8_t)map(theta1+rightHipOffset,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(rightHipServo, pwmVal*10);
    pwmVal = (uint8_t)map(theta2,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(rightKneeServo, pwmVal*10);

    hexy.delay_ms(500);

    pwmVal = (uint8_t)map(-35,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(rightFootServo,pwmVal*10);
    hexy.delay_ms(200);
    pwmVal = (uint8_t)map(90,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(rightFootServo,pwmVal*10);

    hexy.delay_ms(2000);

    pwmVal = (uint8_t)map(0,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(rightHipServo, pwmVal*10);
    hexy.changeServo(rightKneeServo, pwmVal*10);
    hexy.changeServo(rightFootServo, pwmVal*10);
  }
  else
  {
    // left leg
    uint8_t pwmVal = (uint8_t)map(90,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(leftFootServo,pwmVal*10);
    hexy.delay_ms(500);
    
    pwmVal = (uint8_t)map(theta1+leftHipOffset,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(leftHipServo, pwmVal*10);
    pwmVal = (uint8_t)map(theta2,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(leftKneeServo, pwmVal*10);

    hexy.delay_ms(500);

    pwmVal = (uint8_t)map(-35,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(leftFootServo,pwmVal*10);
    hexy.delay_ms(200);
    pwmVal = (uint8_t)map(90,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(leftFootServo,pwmVal*10);

    hexy.delay_ms(2000);

    pwmVal = (uint8_t)map(0,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(leftHipServo, pwmVal*10);
    hexy.changeServo(leftKneeServo, pwmVal*10);
    hexy.changeServo(leftFootServo, pwmVal*10);
  }
  
}

void updateDebounceBuffer(float newData,float bufferArray[], int arraySize)
{
  for (uint8_t index = 0; index < arraySize-1; index++)
  {
    bufferArray[index] = bufferArray[index+1];
  }
  bufferArray[arraySize-1] = newData;
}

int isBufferActive(float bufferArray[], float maximum, float minimum, int arraySize)
{
  float average = 0;
  for (int index = 0; index < arraySize; index++)
  {
    if (bufferArray[index] < minimum || bufferArray[index] > maximum)
    {
      return -1;
    }
    average += bufferArray[index];
  }
  return average/arraySize;
}

bool canHighFive()
{
  return true;
  // TODO, put aactual stuff in.
}



int deltaHead = (lookLeftAngle-lookRightAngle)/timeForFullScan*deltaUpdate;
int desiredHead = 0;
float cm;
const uint8_t BUFFER_SIZE = 5;
float ultrasonicRanges[BUFFER_SIZE] = {100};
void loop() {
  // put your main code here, to run repeatedly:

  //movehead
  uint8_t headPWM = (uint8_t)map(desiredHead,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
  hexy.changeServo(headServo, headPWM*10);


  
  cm = hexy.ping();  
  Serial.print("CM: ");
  Serial.println(cm);
  updateDebounceBuffer(cm,ultrasonicRanges,BUFFER_SIZE);
  
  // detect hand
  float bufferVal = isBufferActive(ultrasonicRanges, ignoreRange, 0.01,BUFFER_SIZE);
  if (bufferVal != -1)
  {
    // Object within 20cm, raise hand  
    if (canHighFive())
    {
      HighFive(bufferVal/100.0,desiredHead);
      hexy.delay_ms(1000);
      //hexy.changeServo(rightFootServo, upPosition*10);
    }
    else if (abs(desiredHead) <= rotateThresh)
    {
      //move forward
    }
    else
    {
      //rotate
    }
    
  }
  else
  {
    // Object further than 20cm or no object at all. Lower hand.
    //hexy.changeServo(rightFootServo, downPosition*10);
  }

  // calculate next head position
  /*if (cm <= 0.01)
  {
    Serial.write("ajghah");
  }*/
  if (cm > 20 || cm <= 0.01) // don't move head when filling buffer of good values
  {
    if (desiredHead <= -90 || desiredHead >= 90)
    {
      deltaHead *= -1;
    }
    desiredHead += deltaHead;
  }
  hexy.delay_ms(deltaUpdate*1000);
}
