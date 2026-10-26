#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#define BUS_MPU9250     1
#define BUS_VL53L1X     6
#define BUS_MS5611      4
#define BUS_INA219      5


#define PWM_MIN 1050
#define PWM_MAX 2200

#define PIN_MOTOR1  12 //front-right - CCW
#define PIN_MOTOR2  13 //rear-right - CW
#define PIN_MOTOR3  18 //rear-left - CCW
#define PIN_MOTOR4  19 //front-left - CW

#define PIN_RED_LED     1
#define PIN_GREEN_LED   0

#define MODE_INDOOR      0 //debugging
#define MODE_OUTDOOR     1
#define MODE_TRAINING    2
#define MODE_TEST        3

#define LOOP_TIMER_MS    4 //4ms, to seconds

#define TRAINING_ROUNDS             5
#define TRAINING_ROUNDS_DURATION_MS 15000
#define TRAINING_MAX_ROUND_DATA  (TRAINING_ROUNDS_DURATION_MS/LOOP_TIMER_MS) //as altitude is updated every 20ms, this buffer is bigger enough to hold all training data

#define READING_UNKNOWN 720

#define MAX_BATTERY_VOLTAGE  12.5
#define MIN_BATTERY_VOLTAGE  10.8
#define CRITICAL_BATTERY_VOLTAGE 10.5
#define CRITICAL_CPU_TEMPERATURE 70 //celus

    //Drone(Flight Controller) status
#define FS_SHUTDOWN         0
#define FS_POWERED_ON       1
#define FS_CALIBRATING      2
#define FS_CALIBRATED       3
#define FS_STOPPING         4
#define FS_STOPPED          5
#define FS_STARTING         6

#define FS_STARTED          10
#define FS_PREFLIGHT_CHK    11
#define FS_TAKING_OFF       12
#define FS_LANDING          13
#define FS_HOME             14

#define FS_HOVERING         20
#define FS_TURN_LEFT        21
#define FS_TURN_RIGHT       22
#define FS_MOVE_FORWARD     23
#define FS_MOVE_BACKWARD    24
#define FS_MOVE_LEFT        25
#define FS_MOVE_RIGHT       26
#define FS_FLY_HIGH         27
#define FS_FLY_LOW          28

#endif

