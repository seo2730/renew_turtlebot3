#ifndef ROMILLION_ROBOT_H_
#define ROMILLION_ROBOT_H_

#define NAME                             "Waffle or Waffle Pi"

#define WHEEL_RADIUS                     0.065           // meter
#define WHEEL_SEPARATION                 0.280           // meter 
#define TURNING_RADIUS                   0.140           // meter 
#define ROBOT_RADIUS                     0.165           // meter 
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#endif  //TURTLEBOT3_BURGER_H_
