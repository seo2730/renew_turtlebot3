# renew_turtlebot3(Not complete)
Using another board instead of OpenCr<br>
I use cortex-M4 board. You can also use Arduino board. I first tested in Arduino Mega2560

# romillion_robot.h
This header file is to define your robot state

    #define NAME                             "Burger"
Define your robot name.<br>

    #define WHEEL_RADIUS                     0.033           // meter
Define your robot wheel radius.<br>

    #define WHEEL_SEPARATION                 0.160           // meter (BURGER : 0.160, WAFFLE : 0.287)
wheel speration = (turning radius)x2<br>

    #define TURNING_RADIUS                   0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
Your robot turning radius(NOT WHEEL RADIUS)<br>

    #define ROBOT_RADIUS                     0.105           // meter (BURGER : 0.105, WAFFLE : 0.220)
Your robot radius + wheel thickness<br>

    #define ENCODER_MIN                      -2147483648     // raw
    #define ENCODER_MAX                      2147483648      // raw
Defing encoder min and max<br>

    #define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
    #define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

    #define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
    #define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 


# romillion_core.h
There are a lot of header files, functions and variables

## f401reMap.h
**f401reMap.h** is the header file of cotex
Reference : https://www.instructables.com/id/Quick-Start-to-STM-Nucleo-on-Arduino-IDE/

## MPU9250.h
**MPU9250.h** is the header file of MPU9250
Reference : https://github.com/bolderflight/MPU9250<br>

# romillion.ino




