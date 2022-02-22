#include <avr/pgmspace.h>
#define MOVE_POINTUPTHENDOWN_SIZE 49
#include "Servotor32.h"

Servotor32 hexy;


const PROGMEM uint16_t move_pointUpThenDown_time[MOVE_POINTUPTHENDOWN_SIZE] = {
0, 2691, 2811, 2859, 2923, 2979, 3002, 3017, 3115, 3139, 3219, 3323, 3451, 3642, 3707, 3730, 3754, 5378, 5419, 5441, 5456, 5470, 5507, 5531, 5554, 5569, 5611, 5683, 5714, 5730, 5803, 5827, 5875, 5897, 5912, 5927, 5963, 6363, 6386, 6467, 6490, 6504, 6627, 6641, 6698, 6779, 6802, 6818, 6833};

const PROGMEM uint8_t move_pointUpThenDown_servo[MOVE_POINTUPTHENDOWN_SIZE] = {
26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26};

const PROGMEM uint8_t move_pointUpThenDown_pos[MOVE_POINTUPTHENDOWN_SIZE] = {
1816 , 150 , 154 , 158 , 162 , 166 , 170 , 174 , 177 , 181 , 184 , 189 , 192 , 195 , 200 , 204 , 209 , 207 , 204 , 202 , 197 , 188 , 185 , 181 , 175 , 171 , 167 , 163 , 159 , 154 , 150 , 145 , 141 , 136 , 132 , 122 , 118 , 114 , 109 , 105 , 102 , 97 , 94 , 92 , 87 , 85 , 79 , 78 , 74 };


void move_pointUpThenDown(){
  int startTime = hexy.millis_new();
  int currentTime = 0;
  int last_update = 0;



  /*
  for(int i=0; i<MOVE_POINTUPTHENDOWN_SIZE; i++){
    delayMicroseconds(10);
    currentTime = hexy.millis_new() - startTime;
    uint16_t move_time = pgm_read_word_near(move_pointUpThenDown_time + i);
    while(currentTime < move_time){
      delayMicroseconds(10);
      currentTime = hexy.millis_new() - startTime;
    }
    uint8_t servo_time = pgm_read_byte_near(move_pointUpThenDown_servo + i);
    uint8_t servo_pos  = pgm_read_byte_near(move_pointUpThenDown_pos + i);
    hexy.changeServo(servo_time, servo_pos*10);
    last_update = currentTime;
  }
  */
}
//Move Size is 196 bytes
//Run this move by using:
// move_pointUpThenDown()



const uint8_t rightFootServo = 26;
const uint8_t upPosition = 209;
const uint8_t downPosition = 74;
void setup() {
  // put your setup code here, to run once:
  hexy.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  float cm;
  cm = hexy.ping();
  
  Serial.print("CM: ");
  Serial.println(cm);
  
  if (cm < 20 && cm != 0)
  {
    // Object within 20cm, raise hand
    //uint8_t servo_time = pgm_read_byte_near(move_pointUpThenDown_servo + i);
    //uint8_t servo_pos  = pgm_read_byte_near(move_pointUpThenDown_pos + i);   
    hexy.changeServo(rightFootServo, upPosition*10);
  }
  else
  {
    // Object further than 20cm or no object at all. Lower hand.
    hexy.changeServo(rightFootServo, downPosition*10);
  }
  hexy.delay_ms(200);

}
