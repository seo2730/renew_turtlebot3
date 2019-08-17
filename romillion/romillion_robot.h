#ifndef ROMILLION_ROBOT_H_
#define ROMILLION_ROBOT_H_

#define NAME                             "Burger"

#define WHEEL_RADIUS                     0.05            // cas_meter
#define WHEEL_SEPARATION                 0.160           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TURNING_RADIUS                   0.090           // meter (CAS: 0.085, BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.150           // meter (CAS:0.150, BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#endif
