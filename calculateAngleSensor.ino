#include <avr/pgmspace.h>
#define MOVE_POINTUPTHENDOWN_SIZE 49
#define MOVE_MOVESENSOR_SIZE 147
#include "Servotor32.h"

Servotor32 hexy;


//Move the servo of ultrasonic sensor 

const PROGMEM uint16_t move_moveSensor_time[MOVE_MOVESENSOR_SIZE] = {
0, 21, 38, 39, 60, 66, 92, 124, 137, 169, 195, 224, 254, 265, 292, 293, 326, 340, 366, 374, 393, 423, 460, 557, 597, 617, 634, 648, 668, 669, 700, 713, 725, 751, 774, 785, 811, 842, 880, 925, 964, 994, 1023, 1047, 1072, 1083, 1109, 1111, 1143, 1158, 1167, 1193, 1216, 1226, 1239, 1248, 1274, 1283, 1306, 1320, 1344, 1373, 1402, 1412, 1442, 1442, 1483, 1524, 1571, 1615, 1624, 1652, 1679, 1709, 1737, 1738, 1777, 1796, 1796, 1837, 1865, 1875, 1917, 1961, 1962, 1999, 2032, 2059, 2088, 2102, 2131, 2133, 2174, 2203, 2231, 2256, 2285, 2286, 2327, 2342, 2371, 2383, 2412, 2443, 2476, 2506, 2534, 2573, 2615, 2664, 2705, 2752, 2752, 2790, 2829, 2876, 2904, 2918, 2944, 2965, 2979, 3008, 3038, 3068, 3098, 3129, 3157, 3215, 3276, 3309, 3320, 3375, 3434, 3478, 3516, 3553, 3597, 3669, 3732, 3773, 3800, 3854, 3897, 4046, 4129, 4265, 5043};

const PROGMEM uint8_t move_moveSensor_servo[MOVE_MOVESENSOR_SIZE] = {
31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31};

const PROGMEM uint8_t move_moveSensor_pos[MOVE_MOVESENSOR_SIZE] = {
241 , 241 , 241 , 241 , 238 , 238 , 235 , 235 , 232 , 231 , 231 , 231 , 231 , 228 , 228 , 227 , 224 , 224 , 224 , 221 , 221 , 221 , 220 , 220 , 220 , 220 , 220 , 219 , 219 , 216 , 214 , 211 , 209 , 207 , 204 , 204 , 202 , 202 , 202 , 200 , 200 , 200 , 197 , 194 , 191 , 191 , 186 , 183 , 183 , 179 , 177 , 174 , 174 , 170 , 170 , 170 , 166 , 166 , 162 , 162 , 158 , 158 , 154 , 154 , 154 , 154 , 150 , 150 , 150 , 150 , 145 , 145 , 141 , 141 , 137 , 137 , 133 , 133 , 133 , 129 , 129 , 129 , 129 , 125 , 125 , 124 , 120 , 117 , 114 , 111 , 111 , 111 , 108 , 105 , 102 , 102 , 97 , 97 , 94 , 91 , 91 , 91 , 89 , 86 , 86 , 84 , 84 , 81 , 81 , 81 , 81 , 81 , 81 , 77 , 77 , 77 , 77 , 74 , 74 , 74 , 74 , 72 , 69 , 69 , 69 , 69 , 69 , 65 , 65 , 65 , 65 , 65 , 61 , 61 , 61 , 61 , 61 , 57 , 57 , 57 , 57 , 57 , 53 , 53 , 53 , 53 , 53  };


void move_moveSensor(){
  int startTime = hexy.millis_new();
  int currentTime = 0;
  int last_update = 0;
  for(int i=0; i<MOVE_MOVESENSOR_SIZE; i++){
    delayMicroseconds(10);
    currentTime = hexy.millis_new() - startTime;
    
    Serial.print("Angle in degrees: ");
    Serial.println(pgm_read_byte_near(move_moveSensor_pos + i)/250.0 * 180.0);
    Serial.println(pgm_read_byte_near(move_moveSensor_pos + i));
    

    uint16_t move_time = pgm_read_word_near(move_moveSensor_time + i);
    while(currentTime < move_time){
      delayMicroseconds(10);
      currentTime = hexy.millis_new() - startTime;
    }
    
    uint8_t servo_time = pgm_read_byte_near(move_moveSensor_servo + i);
    uint8_t servo_pos  = pgm_read_byte_near(move_moveSensor_pos + i);
    hexy.changeServo(servo_time, servo_pos*10);
    last_update = currentTime;
    hexy.delay_ms(200);
  }
}


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
const uint8_t middleServo = 31;
void setup() {
  // put your setup code here, to run once:
  hexy.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  float cm;

  move_moveSensor();

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
