/*
 ============================================================
 Name:Constants.h
 Author: Drew Bell
 Version: 1
 Date Created: 12/06/17
 Description: Header File containing shared constants 
 Code adapted from and based off of skeleton files by Ed Carryer
 ============================================================
 */

#ifndef Constants_H
#define Constants_H

//time
#define ONE_SEC 976

// Encoder for QEI0
    #define E0_TICKS_PER_REV_OUT 464
    #define STEERING_RANGE 2.0      // number of wheel turns from neutral to far left (or right)
    #define ENCODER_0_RANGE (int32_t) (STEERING_RANGE * E0_TICKS_PER_REV_OUT)
    #define ENCODER_0_CENTER   (int16_t) (ENCODER_0_RANGE/2)
    #define ENCODER_0 0

    #define STEERING_MAX_CW                 // max number of ticks clockwise
    #define STEERING_MAX_CCW                 // max number of ticks counter-clockwise

    #define ENCODER_1_RANGE 464
    #define ENCODER_1_CENTER ENCODER_0_RANGE/2
    #define ENCODER_1 1

    #define MAX_12_BITS 4095
    #define HALF_OF_12_BITS    (uint16_t) (MAX_12_BITS / 2)
    #define STEER_TO_12_BIT (4095 * 10 / ENCODER_0_RANGE)


// Steering Defines
    #define STEER_DB_ANGLE 2   // deadback angle around 6 (+3/-3)
    #define STEER_DB_EDGE_CCW  (int16_t) ( -1 * ENCODER_0_RANGE / 360)
    #define STEER_DB_EDGE_CW   (int16_t) ( +1 * ENCODER_0_RANGE / 360)

    #define BASE_RETURN_STRENGTH 40.0
    #define STEERING_SPRING_k 0.5
    
    
    //#define MAX_RTC_DUTY (float) 0.70         // don't allow the PWM output be larger than this %
    #define MOTOR_PWM_PERIOD 1600   // Period = 40MHz clock / 25kHz desired freq.
    #define MIN_RTC_DUTY 0.001f
    #define MAX_RTC_DUTY 0.65f
    #define DEFAULT_RTC_STRENGTH (float) 20.0
    #define RTC_STRENGTH_INC (float) 5.0
        
    #define MAX_INTEGRAL 10000


// THROTTLE DEFINES
    #define THROTTLE_MIN_TICK 180
    #define THROTTLE_MAX_TICK 270
    #define THROTTLE_MIDDLE (uint16_t) ((THROTTLE_MAX_TICK + THROTTLE_MIN_TICK) / 2)
    #define THROTTLE_TICK_RANGE (uint16_t) (THROTTLE_MAX_TICK - THROTTLE_MIN_TICK)
    #define THROTTLE_BIT_CONV  (uint16_t) (4095 / THROTTLE_TICK_RANGE)
    #define THROTTLE_TUNE_FACTOR 200
    


// defines for indexing the motorDriveTable
#define MOTOR_FWD 0 
#define MOTOR_REV 1
#define MOTOR_BRAKE 2
#define MOTOR_OFF 3
#define MOTOR_ENABLE 0
#define IN1 1
#define IN2 2
#define MOTOR_CW   MOTOR_FWD
#define MOTOR_CCW  MOTOR_REV


#endif
 /* Constants_H */




