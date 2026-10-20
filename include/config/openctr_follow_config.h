#ifndef OPENCTR_FOLLOW_CONFIG_H
#define OPENCTR_FOLLOW_CONFIG_H

/* FOLLOW uses AOA location frames received within this many milliseconds. */
#define OPENCTR_FOLLOW_FIX_MAX_AGE_MS 300u

/* FOLLOW uses the distance in the AOA frame directly, in centimeters. */
/* Zero distance is treated as an invalid fix. */
#define OPENCTR_FOLLOW_MIN_VALID_DISTANCE_CM 0u
/* Start reversing below 80 cm; stop reversing at 95 cm. */
#define OPENCTR_FOLLOW_START_REVERSE_CM 80u
#define OPENCTR_FOLLOW_STOP_REVERSE_CM 95u
#define OPENCTR_FOLLOW_START_FORWARD_CM 110u
#define OPENCTR_FOLLOW_STOP_FORWARD_CM 100u
#define OPENCTR_FOLLOW_MAX_DISTANCE_CM 300u

/* Azimuth is in degrees. Turning takes priority over distance control. */
#define OPENCTR_FOLLOW_START_TURN_DEG 12u
#define OPENCTR_FOLLOW_STOP_TURN_DEG 5u

/* Target speeds. The chassis X limit is shared by all control sources. */
#define OPENCTR_FOLLOW_FORWARD_MIN_MM_S 150
#define OPENCTR_FOLLOW_FORWARD_MAX_MM_S 300
#define OPENCTR_FOLLOW_REVERSE_MIN_MM_S 100
#define OPENCTR_FOLLOW_REVERSE_MAX_MM_S 400
#define OPENCTR_FOLLOW_TURN_MIN_MRAD_S 300
#define OPENCTR_FOLLOW_TURN_MAX_MRAD_S 2000
#define OPENCTR_CHASSIS_X_LIMIT_MM_S 2000

/* Outer proportional control; wheel speed has its own encoder PID. */
#define OPENCTR_FOLLOW_FORWARD_KP_MM_S_PER_CM 25u
#define OPENCTR_FOLLOW_REVERSE_KP_MM_S_PER_CM 10u
#define OPENCTR_FOLLOW_TURN_KP_MRAD_S_PER_DEG 50u

/* +1: positive azimuth produces positive yaw; -1 reverses it; 0 stops. */
#ifndef OPENCTR_FOLLOW_AZIMUTH_TO_YAW_SIGN
#define OPENCTR_FOLLOW_AZIMUTH_TO_YAW_SIGN 0
#endif

#if OPENCTR_FOLLOW_AZIMUTH_TO_YAW_SIGN < -1 || OPENCTR_FOLLOW_AZIMUTH_TO_YAW_SIGN > 1
#error "OPENCTR_FOLLOW_AZIMUTH_TO_YAW_SIGN must be -1, 0, or 1"
#endif

#endif
