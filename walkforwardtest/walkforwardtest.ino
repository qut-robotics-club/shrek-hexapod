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
const float ignoreRange = 35;// Any distance above this value should be ignored as it is too far away. measured in cm.
const float rotateThresh = 30;

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


#define MOVE_WALKFORWARD_SIZE 264

const PROGMEM uint16_t move_WalkForward_time[MOVE_WALKFORWARD_SIZE] = {
0, 0, 1, 2, 3, 5, 6, 6, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 12, 170, 170, 171, 171, 172, 172, 173, 173, 173, 174, 174, 175, 176, 176, 176, 177, 177, 177, 339, 339, 339, 340, 340, 340, 342, 342, 342, 343, 343, 343, 344, 344, 344, 345, 345, 345, 608, 608, 609, 611, 612, 613, 614, 615, 616, 616, 617, 617, 617, 617, 618, 618, 618, 618, 619, 619, 619, 619, 620, 620, 620, 621, 621, 621, 621, 622, 781, 782, 782, 783, 783, 784, 784, 785, 785, 786, 786, 786, 786, 786, 787, 787, 787, 787, 950, 950, 951, 951, 952, 952, 953, 953, 953, 954, 954, 955, 955, 955, 955, 956, 956, 956, 1215, 1216, 1216, 1218, 1219, 1219, 1220, 1221, 1222, 1222, 1222, 1222, 1223, 1223, 1223, 1223, 1223, 1224, 1224, 1224, 1224, 1224, 1225, 1225, 1225, 1225, 1226, 1226, 1226, 1226, 1385, 1385, 1386, 1386, 1386, 1386, 1387, 1387, 1388, 1389, 1389, 1389, 1390, 1390, 1390, 1391, 1391, 1391, 1554, 1554, 1555, 1555, 1556, 1556, 1557, 1557, 1557, 1558, 1558, 1558, 1558, 1559, 1559, 1559, 1559, 1560, 1822, 1823, 1823, 1824, 1825, 1826, 1827, 1827, 1828, 1828, 1829, 1829, 1829, 1829, 1830, 1830, 1830, 1830, 1830, 1831, 1831, 1831, 1831, 1831, 1832, 1832, 1832, 1832, 1832, 1832, 1992, 1993, 1993, 1993, 1994, 1994, 1995, 1995, 1996, 1996, 1996, 1997, 1997, 1997, 1998, 1998, 1998, 1998, 2161, 2161, 2162, 2162, 2162, 2163, 2163, 2164, 2164, 2164, 2165, 2165, 2165, 2165, 2166, 2166, 2166, 2166};

const PROGMEM uint8_t move_WalkForward_servo[MOVE_WALKFORWARD_SIZE] = {
6, 6, 5, 21, 5, 21, 7, 14, 22, 7, 14, 24, 22, 13, 24, 11, 20, 13, 24, 11, 16, 20, 15, 24, 16, 11, 15, 16, 11, 16, 6, 6, 5, 5, 7, 7, 21, 21, 22, 22, 20, 14, 20, 14, 13, 13, 15, 15, 6, 6, 5, 5, 7, 7, 21, 21, 22, 22, 20, 20, 14, 14, 13, 13, 15, 15, 25, 25, 10, 26, 10, 17, 26, 17, 9, 24, 7, 9, 18, 24, 7, 20, 11, 18, 7, 15, 20, 11, 16, 7, 15, 20, 16, 15, 20, 15, 25, 25, 26, 26, 10, 24, 10, 24, 9, 17, 9, 17, 11, 18, 11, 18, 16, 16, 25, 25, 26, 26, 24, 24, 10, 10, 9, 17, 9, 17, 11, 18, 11, 18, 16, 16, 6, 6, 5, 21, 5, 21, 14, 7, 22, 14, 7, 22, 24, 13, 11, 20, 24, 13, 16, 11, 20, 24, 15, 16, 11, 24, 15, 16, 11, 16, 6, 6, 5, 5, 7, 7, 21, 21, 22, 22, 14, 20, 14, 20, 13, 13, 15, 15, 6, 6, 5, 5, 7, 7, 21, 21, 22, 22, 20, 20, 14, 14, 13, 13, 15, 15, 25, 25, 10, 26, 10, 26, 9, 17, 24, 7, 9, 17, 24, 20, 7, 11, 18, 15, 20, 11, 7, 15, 18, 20, 7, 15, 16, 20, 15, 16, 25, 25, 26, 26, 24, 24, 10, 10, 9, 17, 9, 17, 11, 18, 11, 18, 16, 16, 25, 25, 26, 26, 24, 24, 10, 10, 9, 9, 17, 11, 17, 11, 18, 18, 16, 16};

const PROGMEM uint8_t move_WalkForward_pos[MOVE_WALKFORWARD_SIZE] = {
209 , 156 , 109 , 209 , 56 , 156 , 205 , 209 , 109 , 187 , 156 , 150 , 56 , 109 , 94 , 122 , 122 , 56 , 94 , 177 , 205 , 140 , 150 , 94 , 150 , 177 , 131 , 150 , 177 , 150 , 156 , 156 , 56 , 56 , 187 , 168 , 156 , 156 , 56 , 56 , 140 , 156 , 159 , 156 , 56 , 56 , 131 , 113 , 156 , 209 , 56 , 109 , 168 , 150 , 156 , 209 , 56 , 109 , 159 , 177 , 156 , 209 , 56 , 109 , 113 , 94 , 209 , 156 , 209 , 109 , 156 , 209 , 56 , 156 , 109 , 94 , 150 , 56 , 109 , 113 , 205 , 177 , 177 , 56 , 205 , 94 , 122 , 159 , 150 , 205 , 150 , 122 , 168 , 150 , 122 , 150 , 156 , 156 , 56 , 56 , 156 , 113 , 156 , 131 , 56 , 156 , 56 , 156 , 159 , 56 , 140 , 56 , 168 , 187 , 156 , 209 , 56 , 109 , 131 , 150 , 156 , 209 , 56 , 156 , 109 , 209 , 140 , 56 , 122 , 109 , 187 , 205 , 209 , 156 , 109 , 209 , 56 , 156 , 209 , 205 , 109 , 156 , 187 , 56 , 150 , 109 , 122 , 122 , 94 , 56 , 205 , 177 , 140 , 94 , 150 , 150 , 177 , 94 , 131 , 150 , 177 , 150 , 156 , 156 , 56 , 56 , 187 , 168 , 156 , 156 , 56 , 56 , 156 , 140 , 156 , 159 , 56 , 56 , 131 , 113 , 156 , 209 , 56 , 109 , 168 , 150 , 156 , 209 , 56 , 109 , 159 , 177 , 156 , 209 , 56 , 109 , 113 , 94 , 209 , 156 , 209 , 109 , 156 , 56 , 109 , 209 , 94 , 150 , 56 , 156 , 113 , 177 , 205 , 177 , 109 , 94 , 122 , 159 , 205 , 150 , 56 , 122 , 205 , 150 , 150 , 122 , 150 , 168 , 156 , 156 , 56 , 56 , 113 , 131 , 156 , 156 , 56 , 156 , 56 , 156 , 159 , 56 , 140 , 56 , 168 , 187 , 156 , 209 , 56 , 109 , 131 , 150 , 156 , 209 , 56 , 109 , 156 , 140 , 209 , 122 , 56 , 109 , 187 , 205 };


void WalkForward(float fraction){
  
  int startTime = hexy.millis_new();
  int currentTime = 0;
  int last_update = 0;
  for(int i=0; i<MOVE_WALKFORWARD_SIZE*fraction; i++){
    delayMicroseconds(10);
    currentTime = hexy.millis_new() - startTime;
    uint16_t move_time = pgm_read_word_near(move_WalkForward_time + i);
    while(currentTime < move_time){
      delayMicroseconds(10);
      currentTime = hexy.millis_new() - startTime;
    }
    uint8_t servo_time = pgm_read_byte_near(move_WalkForward_servo + i);
    uint8_t servo_pos  = pgm_read_byte_near(move_WalkForward_pos + i);
    hexy.changeServo(servo_time, servo_pos*10);
    last_update = currentTime;
  }
}
//Move Size is 1056 bytes
//Run this move by using:
// move_WalkForward()
void WalkForwardOffset(int hipOffsets){
  
  int startTime = hexy.millis_new();
  int currentTime = 0;
  int last_update = 0;
  for(int i=0; i<MOVE_WALKFORWARD_SIZE; i++){
    delayMicroseconds(10);
    currentTime = hexy.millis_new() - startTime;
    uint16_t move_time = pgm_read_word_near(move_WalkForward_time + i);
    while(currentTime < move_time){
      delayMicroseconds(10);
      currentTime = hexy.millis_new() - startTime;
    }
    uint8_t servo_time = pgm_read_byte_near(move_WalkForward_servo + i);
    uint8_t servo_pos  = pgm_read_byte_near(move_WalkForward_pos + i);
    hexy.changeServo(servo_time, servo_pos*10);
    last_update = currentTime;
  }
}


#define MOVE_GETUP_SIZE 570

const PROGMEM uint16_t move_GetUp_time[MOVE_GETUP_SIZE] = {
0, 0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 505, 506, 506, 507, 508, 508, 509, 509, 509, 510, 510, 510, 511, 511, 512, 512, 513, 513, 1014, 1015, 1015, 1016, 1016, 1016, 1017, 1017, 1017, 1018, 1018, 1018, 1518, 1519, 1520, 1520, 1521, 1521, 1521, 1522, 1522, 1522, 1523, 1523, 1523, 1524, 1524, 1524, 1525, 1525, 1525, 1526, 1526, 1526, 1526, 1527, 1626, 1626, 1627, 1627, 1628, 1628, 1629, 1629, 1630, 1630, 1631, 1631, 1631, 1632, 1632, 1632, 1633, 1633, 1634, 1634, 1634, 1634, 1635, 1635, 1735, 1735, 1735, 1735, 1736, 1736, 1736, 1736, 1737, 1737, 1738, 1738, 1738, 1738, 1739, 1739, 1740, 1740, 1740, 1740, 1741, 1741, 1741, 1742, 1842, 1843, 1843, 1843, 1844, 1844, 1844, 1845, 1845, 1845, 1846, 1846, 1846, 1847, 1847, 1847, 1848, 1848, 1849, 1849, 1849, 1849, 1850, 1850, 1951, 1951, 1952, 1952, 1952, 1953, 1953, 1953, 1954, 1954, 1954, 1954, 1955, 1955, 1955, 1956, 1956, 1956, 1956, 1957, 1957, 1957, 1958, 1958, 2058, 2058, 2059, 2059, 2059, 2059, 2060, 2060, 2060, 2061, 2061, 2061, 2061, 2062, 2062, 2062, 2063, 2063, 2063, 2064, 2064, 2064, 2064, 2065, 2166, 2166, 2166, 2167, 2167, 2167, 2168, 2168, 2168, 2169, 2169, 2169, 2170, 2170, 2170, 2171, 2171, 2171, 2172, 2172, 2172, 2172, 2173, 2173, 2273, 2274, 2274, 2274, 2275, 2275, 2275, 2275, 2276, 2276, 2276, 2277, 2277, 2277, 2278, 2278, 2278, 2279, 2279, 2280, 2280, 2280, 2280, 2281, 2381, 2381, 2381, 2382, 2382, 2382, 2383, 2383, 2383, 2384, 2384, 2384, 2384, 2385, 2385, 2385, 2386, 2386, 2386, 2386, 2387, 2387, 2387, 2387, 2489, 2489, 2489, 2490, 2490, 2490, 2491, 2491, 2491, 2492, 2492, 2492, 2493, 2493, 2493, 2494, 2494, 2494, 2495, 2495, 2495, 2496, 2496, 2496, 2596, 2596, 2597, 2597, 2597, 2597, 2598, 2598, 2598, 2599, 2599, 2600, 2600, 2600, 2601, 2601, 2601, 2602, 2602, 2602, 2603, 2603, 2603, 2603, 2704, 2704, 2705, 2705, 2705, 2706, 2706, 2706, 2706, 2707, 2707, 2707, 2708, 2708, 2708, 2709, 2709, 2710, 2710, 2710, 2710, 2711, 2711, 2711, 2813, 2813, 2813, 2814, 2814, 2814, 2815, 2815, 2815, 2816, 2816, 2816, 2817, 2817, 2817, 2818, 2818, 2818, 2819, 2819, 2819, 2820, 2820, 2820, 2921, 2922, 2922, 2922, 2922, 2923, 2923, 2924, 2924, 2924, 2924, 2925, 2925, 2925, 2926, 2926, 2926, 2927, 2927, 2927, 2928, 2928, 2928, 2929, 3029, 3029, 3030, 3030, 3031, 3031, 3031, 3032, 3032, 3032, 3033, 3033, 3033, 3034, 3034, 3034, 3035, 3035, 3035, 3036, 3036, 3036, 3037, 3037, 3143, 3144, 3145, 3146, 3146, 3146, 3147, 3147, 3147, 3147, 3147, 3147, 3547, 3547, 3548, 3549, 3549, 3550, 3550, 3550, 3551, 3551, 3551, 3551, 3552, 3552, 3552, 3552, 3553, 3553, 3650, 3650, 3650, 3651, 3651, 3652, 3652, 3653, 3653, 3654, 3654, 3654, 3654, 3654, 3655, 3655, 3655, 3655, 3751, 3751, 3752, 3752, 3752, 3753, 3753, 3753, 3754, 3754, 3755, 3755, 3755, 3755, 3756, 3756, 3756, 3757, 4050, 4051, 4051, 4052, 4053, 4053, 4053, 4053, 4053, 4053, 4053, 4053, 4453, 4454, 4454, 4455, 4456, 4456, 4457, 4457, 4457, 4457, 4458, 4458, 4458, 4458, 4458, 4459, 4459, 4459, 4556, 4556, 4557, 4557, 4558, 4558, 4559, 4559, 4559, 4560, 4560, 4560, 4560, 4561, 4561, 4561, 4562, 4562, 4658, 4658, 4658, 4659, 4659, 4659, 4660, 4660, 4660, 4661, 4661, 4662, 4662, 4663, 4663, 4663, 4663, 4664, 4960, 4961, 4961, 4963, 4964, 4965, 4965, 4966, 4967, 4967, 4968, 4969, 4969, 4969, 4970, 4970, 4971, 4971, 4971, 4972, 4972, 4972, 4973, 4974, 4974, 4975, 4975, 4975, 4975, 4976, 4976, 4976, 4977, 4977, 4977, 4978};

const PROGMEM uint8_t move_GetUp_servo[MOVE_GETUP_SIZE] = {
7, 7, 20, 20, 15, 15, 24, 24, 11, 11, 16, 16, 25, 25, 24, 21, 21, 20, 17, 17, 16, 6, 6, 7, 10, 10, 11, 14, 14, 15, 26, 26, 22, 22, 18, 18, 5, 5, 9, 9, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 25, 25, 26, 26, 21, 21, 22, 22, 17, 17, 18, 18, 6, 6, 5, 5, 10, 10, 9, 9, 14, 14, 13, 13, 6, 6, 21, 5, 21, 5, 14, 22, 14, 22, 13, 13, 6, 6, 5, 21, 5, 21, 14, 7, 22, 14, 7, 22, 13, 20, 13, 20, 15, 15, 6, 6, 5, 5, 7, 21, 14, 7, 21, 14, 22, 13, 22, 13, 20, 15, 20, 15, 6, 6, 5, 5, 7, 7, 21, 21, 22, 14, 22, 20, 20, 14, 13, 13, 15, 15, 25, 25, 10, 26, 10, 26, 17, 9, 17, 9, 18, 18, 25, 25, 10, 26, 17, 26, 10, 17, 24, 9, 18, 24, 9, 18, 11, 16, 11, 16, 25, 25, 26, 26, 17, 10, 24, 17, 10, 24, 18, 9, 18, 9, 16, 11, 16, 11, 25, 25, 26, 26, 24, 24, 10, 17, 10, 17, 9, 18, 9, 18, 11, 16, 11, 16, 6, 6, 10, 5, 10, 14, 5, 9, 14, 9, 25, 13, 25, 21, 13, 26, 21, 17, 7, 26, 22, 17, 7, 22, 18, 20, 18, 20, 15, 15, 24, 24, 11, 11, 16, 16};

const PROGMEM uint8_t move_GetUp_pos[MOVE_GETUP_SIZE] = {
183 , 183 , 150 , 151 , 116 , 116 , 116 , 116 , 150 , 151 , 183 , 183 , 209 , 116 , 116 , 209 , 116 , 151 , 209 , 116 , 183 , 209 , 116 , 183 , 209 , 116 , 151 , 209 , 116 , 116 , 109 , 50 , 109 , 50 , 109 , 50 , 109 , 50 , 109 , 50 , 109 , 50 , 116 , 150 , 50 , 50 , 116 , 150 , 50 , 50 , 116 , 150 , 50 , 50 , 116 , 150 , 50 , 50 , 116 , 150 , 50 , 50 , 116 , 150 , 50 , 50 , 150 , 153 , 50 , 53 , 150 , 153 , 50 , 53 , 150 , 153 , 50 , 53 , 150 , 153 , 50 , 53 , 150 , 153 , 50 , 53 , 150 , 153 , 50 , 53 , 153 , 156 , 53 , 56 , 153 , 156 , 53 , 56 , 153 , 156 , 53 , 56 , 153 , 156 , 53 , 56 , 153 , 156 , 53 , 56 , 153 , 156 , 53 , 56 , 156 , 160 , 56 , 60 , 156 , 160 , 56 , 60 , 156 , 160 , 56 , 60 , 156 , 160 , 56 , 60 , 156 , 160 , 56 , 60 , 156 , 160 , 56 , 60 , 160 , 163 , 60 , 63 , 160 , 163 , 60 , 63 , 160 , 163 , 60 , 63 , 160 , 163 , 60 , 63 , 160 , 163 , 60 , 63 , 160 , 163 , 60 , 63 , 163 , 166 , 63 , 66 , 163 , 166 , 63 , 66 , 163 , 166 , 63 , 66 , 163 , 166 , 63 , 66 , 163 , 166 , 63 , 66 , 163 , 166 , 63 , 66 , 166 , 170 , 66 , 70 , 166 , 170 , 66 , 70 , 166 , 170 , 66 , 70 , 166 , 170 , 66 , 70 , 166 , 170 , 66 , 70 , 166 , 170 , 66 , 70 , 170 , 173 , 70 , 73 , 170 , 173 , 70 , 73 , 170 , 173 , 70 , 73 , 170 , 173 , 70 , 73 , 170 , 173 , 70 , 73 , 170 , 173 , 70 , 73 , 173 , 176 , 73 , 76 , 173 , 176 , 73 , 76 , 173 , 176 , 73 , 76 , 173 , 176 , 73 , 76 , 173 , 176 , 73 , 76 , 173 , 176 , 73 , 76 , 176 , 180 , 76 , 80 , 176 , 180 , 76 , 80 , 176 , 180 , 76 , 80 , 176 , 180 , 76 , 80 , 176 , 180 , 76 , 80 , 176 , 180 , 76 , 80 , 180 , 183 , 80 , 83 , 180 , 183 , 80 , 83 , 180 , 183 , 80 , 83 , 180 , 183 , 80 , 83 , 180 , 183 , 80 , 83 , 180 , 183 , 80 , 83 , 183 , 186 , 83 , 86 , 183 , 186 , 83 , 86 , 183 , 186 , 83 , 86 , 183 , 186 , 83 , 86 , 183 , 186 , 83 , 86 , 183 , 186 , 83 , 86 , 186 , 190 , 86 , 90 , 186 , 190 , 86 , 90 , 186 , 190 , 86 , 90 , 186 , 190 , 86 , 90 , 186 , 190 , 86 , 90 , 186 , 190 , 86 , 90 , 190 , 193 , 90 , 93 , 190 , 193 , 90 , 93 , 190 , 193 , 90 , 93 , 190 , 193 , 90 , 93 , 190 , 193 , 90 , 93 , 190 , 193 , 90 , 93 , 193 , 196 , 93 , 96 , 193 , 196 , 93 , 96 , 193 , 196 , 93 , 96 , 193 , 196 , 93 , 96 , 193 , 196 , 93 , 96 , 193 , 196 , 93 , 96 , 196 , 150 , 196 , 96 , 150 , 50 , 196 , 96 , 150 , 50 , 96 , 50 , 150 , 209 , 50 , 150 , 109 , 156 , 150 , 183 , 50 , 209 , 183 , 56 , 50 , 151 , 109 , 150 , 116 , 116 , 209 , 209 , 109 , 109 , 183 , 156 , 209 , 183 , 156 , 209 , 56 , 109 , 56 , 109 , 150 , 116 , 150 , 116 , 209 , 209 , 109 , 109 , 183 , 183 , 156 , 209 , 56 , 209 , 109 , 150 , 150 , 209 , 109 , 109 , 116 , 116 , 196 , 150 , 196 , 96 , 150 , 50 , 196 , 96 , 150 , 50 , 96 , 50 , 150 , 209 , 150 , 50 , 150 , 109 , 156 , 209 , 116 , 50 , 50 , 116 , 56 , 109 , 151 , 183 , 150 , 183 , 209 , 209 , 109 , 109 , 209 , 156 , 116 , 209 , 156 , 116 , 109 , 56 , 109 , 56 , 183 , 150 , 183 , 150 , 209 , 209 , 109 , 109 , 116 , 116 , 156 , 209 , 209 , 209 , 56 , 109 , 109 , 109 , 150 , 183 , 150 , 183 , 209 , 209 , 209 , 109 , 209 , 209 , 109 , 109 , 209 , 109 , 209 , 109 , 209 , 209 , 109 , 109 , 209 , 209 , 183 , 109 , 109 , 209 , 183 , 109 , 109 , 150 , 109 , 150 , 116 , 116 , 116 , 116 , 150 , 150 , 183 , 183 };
// 7-183,5-109,6-109

void GetUp(){
  int startTime = hexy.millis_new();
  int currentTime = 0;
  int last_update = 0;
  for(int i=0; i<MOVE_GETUP_SIZE; i++){
    delayMicroseconds(10);
    currentTime = hexy.millis_new() - startTime;
    uint16_t move_time = pgm_read_word_near(move_GetUp_time + i);
    while(currentTime < move_time){
      delayMicroseconds(10);
      currentTime = hexy.millis_new() - startTime;
    }
    uint8_t servo_time = pgm_read_byte_near(move_GetUp_servo + i);
    uint8_t servo_pos  = pgm_read_byte_near(move_GetUp_pos + i);
    hexy.changeServo(servo_time, servo_pos*10);
    last_update = currentTime;
  }
}
//Move Size is 2280 bytes
//Run this move by using:
// move_GetUp()



#define MOVE_ROTATELEFT_SIZE 174

const PROGMEM uint16_t move_RotateLeft_time[MOVE_ROTATELEFT_SIZE] = {
0, 1, 2, 2, 3, 5, 5, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 70, 70, 71, 71, 71, 71, 72, 72, 73, 73, 74, 74, 74, 75, 75, 75, 76, 76, 139, 139, 139, 140, 140, 140, 141, 141, 141, 142, 142, 143, 143, 143, 144, 144, 144, 144, 303, 304, 304, 305, 305, 306, 306, 306, 307, 307, 307, 307, 607, 608, 611, 612, 612, 613, 1014, 1015, 1015, 1016, 1016, 1017, 1017, 1017, 1018, 1018, 1018, 1018, 1317, 1318, 1319, 1319, 1320, 1320, 1321, 1321, 1322, 1322, 1322, 1322, 1322, 1323, 1323, 1323, 1323, 1323, 1420, 1420, 1421, 1421, 1422, 1422, 1423, 1423, 1424, 1424, 1424, 1424, 1425, 1425, 1425, 1425, 1425, 1426, 1521, 1522, 1522, 1522, 1523, 1523, 1523, 1524, 1524, 1525, 1525, 1525, 1526, 1526, 1526, 1527, 1527, 1527, 1824, 1825, 1825, 1826, 1827, 1828, 1828, 1829, 1829, 1830, 1830, 1831, 1832, 1832, 1832, 1832, 1833, 1833, 1833, 1834, 1834, 1834, 1835, 1835, 1835, 1836, 1836, 1836, 1837, 1837, 1837, 1838, 1838, 1838, 1838, 1839};

const PROGMEM uint8_t move_RotateLeft_servo[MOVE_ROTATELEFT_SIZE] = {
6, 6, 14, 5, 14, 5, 21, 13, 7, 21, 13, 7, 22, 15, 22, 15, 20, 20, 6, 6, 5, 5, 7, 7, 14, 14, 13, 21, 13, 21, 15, 22, 15, 22, 20, 20, 6, 6, 5, 5, 7, 7, 14, 14, 13, 13, 21, 15, 21, 15, 22, 22, 20, 20, 25, 25, 17, 26, 26, 17, 10, 18, 10, 18, 9, 9, 7, 7, 15, 15, 20, 20, 25, 25, 26, 17, 26, 17, 10, 18, 10, 18, 9, 9, 6, 6, 5, 21, 5, 21, 7, 14, 22, 7, 14, 22, 13, 20, 13, 20, 15, 15, 6, 6, 5, 5, 7, 7, 14, 21, 14, 21, 13, 22, 13, 22, 15, 20, 15, 20, 6, 6, 5, 5, 7, 7, 14, 14, 13, 13, 21, 15, 21, 15, 22, 22, 20, 20, 6, 6, 10, 5, 10, 5, 9, 14, 9, 14, 13, 13, 25, 21, 25, 21, 17, 7, 26, 22, 17, 26, 7, 22, 18, 20, 18, 20, 15, 15, 24, 24, 11, 11, 16, 16};

const PROGMEM uint8_t move_RotateLeft_pos[MOVE_ROTATELEFT_SIZE] = {
209 , 156 , 209 , 109 , 156 , 56 , 209 , 109 , 205 , 156 , 56 , 201 , 109 , 150 , 56 , 164 , 122 , 146 , 156 , 156 , 56 , 56 , 201 , 198 , 156 , 156 , 56 , 156 , 56 , 156 , 164 , 56 , 179 , 56 , 146 , 170 , 156 , 209 , 56 , 109 , 198 , 194 , 156 , 209 , 56 , 109 , 156 , 179 , 209 , 194 , 56 , 109 , 170 , 194 , 209 , 176 , 209 , 109 , 76 , 176 , 209 , 109 , 176 , 76 , 109 , 76 , 194 , 105 , 194 , 105 , 194 , 105 , 176 , 209 , 76 , 176 , 109 , 209 , 176 , 76 , 209 , 109 , 76 , 109 , 209 , 156 , 109 , 209 , 56 , 156 , 105 , 209 , 109 , 135 , 209 , 56 , 109 , 105 , 109 , 120 , 105 , 105 , 156 , 156 , 56 , 56 , 135 , 164 , 209 , 156 , 209 , 156 , 109 , 56 , 109 , 56 , 105 , 120 , 105 , 135 , 156 , 209 , 56 , 109 , 164 , 194 , 209 , 209 , 109 , 109 , 156 , 105 , 209 , 105 , 56 , 109 , 135 , 151 , 209 , 209 , 209 , 109 , 209 , 109 , 109 , 209 , 109 , 209 , 109 , 109 , 209 , 209 , 209 , 209 , 209 , 194 , 109 , 109 , 209 , 109 , 183 , 109 , 109 , 151 , 109 , 150 , 105 , 116 , 150 , 116 , 122 , 150 , 205 , 183 };


void RotateLeft(){
  int startTime = hexy.millis_new();
  int currentTime = 0;
  int last_update = 0;
  for(int i=0; i<MOVE_ROTATELEFT_SIZE; i++){
    delayMicroseconds(10);
    currentTime = hexy.millis_new() - startTime;
    uint16_t move_time = pgm_read_word_near(move_RotateLeft_time + i);
    while(currentTime < move_time){
      delayMicroseconds(10);
      currentTime = hexy.millis_new() - startTime;
    }
    uint8_t servo_time = pgm_read_byte_near(move_RotateLeft_servo + i);
    uint8_t servo_pos  = pgm_read_byte_near(move_RotateLeft_pos + i);
    hexy.changeServo(servo_time, servo_pos*10);
    last_update = currentTime;
  }
}
//Move Size is 696 bytes
//Run this move by using:
// move_RotateLeft()


#define MOVE_ROTATERIGHT_SIZE 174

const PROGMEM uint16_t move_RotateRight_time[MOVE_ROTATERIGHT_SIZE] = {
0, 0, 1, 2, 3, 3, 3, 4, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 70, 70, 71, 71, 71, 71, 72, 72, 73, 73, 74, 74, 74, 75, 75, 75, 75, 75, 137, 138, 138, 138, 139, 139, 140, 141, 141, 142, 142, 142, 143, 143, 143, 144, 144, 144, 503, 504, 505, 506, 506, 507, 507, 508, 508, 508, 508, 508, 808, 808, 809, 810, 810, 811, 1214, 1214, 1215, 1216, 1217, 1217, 1217, 1218, 1218, 1218, 1218, 1218, 1518, 1519, 1519, 1520, 1521, 1521, 1522, 1522, 1522, 1523, 1523, 1523, 1523, 1524, 1524, 1524, 1524, 1524, 1621, 1621, 1622, 1622, 1623, 1623, 1624, 1624, 1625, 1625, 1625, 1625, 1626, 1626, 1626, 1626, 1627, 1627, 1723, 1723, 1723, 1724, 1724, 1725, 1726, 1726, 1727, 1727, 1727, 1728, 1728, 1728, 1729, 1729, 1729, 1729, 1825, 1826, 1827, 1828, 1828, 1829, 1829, 1830, 1831, 1832, 1832, 1833, 1833, 1834, 1834, 1835, 1835, 1835, 1836, 1836, 1836, 1837, 1837, 1837, 1838, 1838, 1839, 1839, 1839, 1840, 1840, 1840, 1840, 1841, 1841, 1841};

const PROGMEM uint8_t move_RotateRight_servo[MOVE_ROTATERIGHT_SIZE] = {
25, 25, 26, 17, 26, 10, 17, 24, 10, 18, 24, 9, 18, 9, 16, 11, 16, 11, 25, 25, 26, 26, 24, 24, 10, 17, 10, 17, 9, 18, 9, 11, 18, 11, 16, 16, 25, 25, 26, 26, 24, 24, 10, 10, 9, 9, 17, 11, 17, 11, 18, 18, 16, 16, 6, 6, 5, 14, 5, 14, 21, 13, 21, 13, 22, 22, 24, 24, 16, 16, 11, 11, 6, 6, 5, 14, 5, 14, 21, 13, 21, 13, 22, 22, 25, 25, 26, 10, 26, 10, 17, 24, 9, 17, 24, 9, 18, 11, 18, 11, 16, 16, 25, 25, 26, 26, 10, 24, 10, 24, 9, 17, 9, 17, 11, 18, 11, 18, 16, 16, 25, 25, 26, 26, 10, 24, 10, 24, 17, 9, 17, 9, 18, 11, 18, 11, 16, 16, 6, 6, 5, 10, 5, 10, 9, 14, 9, 14, 13, 25, 13, 25, 21, 26, 21, 17, 26, 7, 22, 17, 7, 22, 18, 20, 18, 20, 15, 15, 24, 24, 11, 11, 16, 16};

const PROGMEM uint8_t move_RotateRight_pos[MOVE_ROTATERIGHT_SIZE] = {
209 , 156 , 109 , 209 , 56 , 209 , 156 , 116 , 156 , 109 , 113 , 109 , 56 , 56 , 183 , 150 , 157 , 135 , 156 , 156 , 56 , 56 , 113 , 109 , 156 , 156 , 156 , 156 , 56 , 56 , 56 , 135 , 56 , 120 , 157 , 131 , 156 , 209 , 56 , 109 , 109 , 105 , 156 , 209 , 56 , 109 , 156 , 120 , 209 , 105 , 56 , 109 , 131 , 105 , 209 , 176 , 109 , 209 , 76 , 176 , 209 , 109 , 176 , 76 , 109 , 76 , 105 , 194 , 105 , 194 , 105 , 194 , 176 , 209 , 76 , 176 , 109 , 209 , 176 , 76 , 209 , 109 , 76 , 109 , 209 , 156 , 109 , 209 , 56 , 156 , 209 , 194 , 109 , 209 , 164 , 56 , 109 , 194 , 109 , 180 , 194 , 194 , 156 , 156 , 56 , 56 , 156 , 164 , 156 , 135 , 56 , 209 , 56 , 209 , 180 , 109 , 165 , 109 , 194 , 194 , 156 , 209 , 56 , 109 , 156 , 135 , 209 , 105 , 209 , 56 , 209 , 109 , 109 , 165 , 109 , 151 , 194 , 194 , 209 , 209 , 109 , 209 , 109 , 209 , 109 , 209 , 109 , 209 , 109 , 209 , 109 , 209 , 209 , 109 , 209 , 209 , 109 , 183 , 109 , 209 , 183 , 109 , 109 , 150 , 109 , 150 , 116 , 116 , 105 , 116 , 151 , 150 , 194 , 183 };


void RotateRight(){
  int startTime = hexy.millis_new();
  int currentTime = 0;
  int last_update = 0;
  for(int i=0; i<MOVE_ROTATERIGHT_SIZE; i++){
    delayMicroseconds(10);
    currentTime = hexy.millis_new() - startTime;
    uint16_t move_time = pgm_read_word_near(move_RotateRight_time + i);
    while(currentTime < move_time){
      delayMicroseconds(10);
      currentTime = hexy.millis_new() - startTime;
    }
    uint8_t servo_time = pgm_read_byte_near(move_RotateRight_servo + i);
    uint8_t servo_pos  = pgm_read_byte_near(move_RotateRight_pos + i);
    hexy.changeServo(servo_time, servo_pos*10);
    last_update = currentTime;
  }
}
//Move Size is 696 bytes
//Run this move by using:
// move_RotateRight()









void setup() {
  // put your setup code here, to run once:
  hexy.begin();
  GetUp();
  /*
  uint8_t pwmVal = (uint8_t)map(0,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
  hexy.changeServo(rightHipServo, pwmVal*10);  
  hexy.changeServo(rightKneeServo, pwmVal*10);
  hexy.changeServo(rightFootServo, pwmVal*10);
  // left leg
  hexy.changeServo(leftFootServo, pwmVal*10);
  hexy.changeServo(leftHipServo, pwmVal*10);
  hexy.changeServo(leftKneeServo, pwmVal*10);
  */
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

/*
  Serial.println("Heading = " + String(heading));
  Serial.println("Range = " + String(range));
  Serial.println("X = " + String(X));
  Serial.println("Y = " + String(Y));
  Serial.println("x = " + String(x));
  Serial.println("y = " + String(y));
  Serial.println("theta1 = " + String(theta1));
  Serial.println("theta2 = " + String(theta2));
  Serial.println("________");
*/
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

    hexy.delay_ms(1000);

    pwmVal = (uint8_t)map(-35,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(rightFootServo,pwmVal*10);
    hexy.delay_ms(200);
    pwmVal = (uint8_t)map(90,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(rightFootServo,pwmVal*10);

    hexy.delay_ms(2000);

    pwmVal = (uint8_t)map(0,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
//    hexy.changeServo(rightHipServo, pwmVal*10);
//    hexy.changeServo(rightKneeServo, pwmVal*10);
//    hexy.changeServo(rightFootServo, pwmVal*10);
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

    hexy.delay_ms(1000);

    pwmVal = (uint8_t)map(-35,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(leftFootServo,pwmVal*10);
    hexy.delay_ms(200);
    pwmVal = (uint8_t)map(90,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
    hexy.changeServo(leftFootServo,pwmVal*10);

    hexy.delay_ms(2000);

    pwmVal = (uint8_t)map(0,lookRightAngle,lookLeftAngle,lookLeftPos,lookRightPos);
//    hexy.changeServo(leftHipServo, pwmVal*10);
//    hexy.changeServo(leftKneeServo, pwmVal*10);
//    hexy.changeServo(leftFootServo, pwmVal*10);
  }

  hexy.changeServo(5,1090);
  hexy.changeServo(6,2100);
  hexy.changeServo(7,1830);   
  hexy.changeServo(24,1030);
  hexy.changeServo(25,2100);
  hexy.changeServo(26,1090);
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

bool canHighFive(float range,float heading)
{
  Serial.println(range);

  // convert polar to cartesian, with origin centered at head rotation axis
  range = range + SENSOR_REDUCTION - WRIST_REDUCTION;
  float X = range*cosd(heading);
  float Y = range*sind(heading);

  // translate coordinate system to be centered on a hip.
  float x = X - ARM_FORWARD_OFFSET;
  float y = Y < 0 ? Y+ARM_SIDE_OFFSET:Y-ARM_SIDE_OFFSET;
  
  // calculate IK
  float r = sqrt(sq(x)+sq(y));

  
  return r <= 12.5;
  
}

void FillArray(float bufferArray[],float val,int arraySize)
{
  for (int index = 0; index < arraySize; index++)
  {
    bufferArray[index] = val;
  }
}



int deltaHead = (lookLeftAngle-lookRightAngle)/timeForFullScan*deltaUpdate;
int desiredHead = 0;
float cm;
const uint8_t BUFFER_SIZE = 5;
float ultrasonicRanges[BUFFER_SIZE] = {100};
void loop() {
  // put your main code here, to run repeatedly:
  WalkForward(0.5);
  hexy.changeServo(25,2100);
  hexy.changeServo(6,2100);
  hexy.delay_ms(10*1000);
}
