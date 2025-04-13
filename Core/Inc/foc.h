#ifndef FOC_H
#define FOC_H

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "math.h"
#include "adc.h"

#define TIMER_COUNTER_PERIOD 1000
#define POLE_PAIR_NUM 3
#define DIR -1
#define Tf 1e3f // in macroseconds

extern float exp_velocity;
extern float velocity;
extern float angle;
extern float q, d;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
extern float sensorAngle;
extern int turn_count;
extern float sensorVelocity;
extern float filtered_velocity;
void set_motor(float para_q, float para_d, float para_velocity);
void foc_init();
void set_output();
float PID(float Kp, float Ki, float Kd, float error);

#endif