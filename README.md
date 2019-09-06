# renew_turtlebot3(Not complete)
Using another board instead of OpenCr<br>
I use cortex-M4 board. You can also use Arduino board. I first tested in Arduino Mega2560

# romillion_robot.h
This header file is to define your robot state.<br><br>
Define your robot name.

    #define NAME                   "Burger"
Define your robot wheel radius.<br>

    #define WHEEL_RADIUS           0.033           // meter
wheel speration = (turning radius)x2<br>

    #define WHEEL_SEPARATION       0.160           // meter (BURGER : 0.160, WAFFLE : 0.287)
Your robot turning radius(NOT WHEEL RADIUS)<br>

    #define TURNING_RADIUS         0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
Your robot radius + wheel thickness<br>

    #define ROBOT_RADIUS           0.105           // meter (BURGER : 0.105, WAFFLE : 0.220)
Define min and max of encoder, linear velocity and angular velocity<br>

    #define ENCODER_MIN            -2147483648     // raw
    #define ENCODER_MAX            2147483648      // raw

    #define MAX_LINEAR_VELOCITY    (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
    #define MAX_ANGULAR_VELOCITY   (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

    #define MIN_LINEAR_VELOCITY     -MAX_LINEAR_VELOCITY  
    #define MIN_ANGULAR_VELOCITY    -MAX_ANGULAR_VELOCITY 


# romillion_core.h
There are a lot of header files, functions and variables.<br>

### f401reMap.h
**f401reMap.h** is the header file of cortex<br>
Reference : https://www.instructables.com/id/Quick-Start-to-STM-Nucleo-on-Arduino-IDE/

### MOTOR
Motor : Encoder motor GM36F-3657E DC12V<br>
Motor drive : DMC-16<br>

DMC-16 cw,ccw,pwm pin define

    #define cw_R 7
    #define ccw_R 6
    #define pwm_R 5
    #define cw_L 11
    #define ccw_L 10
    #define pwm_L 9
DMC-16 FG1(encoder A),FG2(encoder B)
 
    int encoder0PinA_L= pinMap(18);
    int encoder0PinB_L = pinMap(19);
    int encoder0PinA_R = pinMap(16);
    int encoder0PinB_R = pinMap(17);
Encoder variables

    int encoder0Pos_R = 0,encoder0Pos_L = 0;
    int prev_encoder_R=0, prev_encoder_L=0;
    const float ratio =360./56./26.;//1456 pulse
PWM variables

    int speed_R,speed_L;
To define cw,ccw direction

    bool ccw_R_dir,cw_R_dir;
    bool ccw_L_dir,cw_L_dir;
### MPU9250.h
**MPU9250.h** is the header file of MPU9250<br>
Reference : https://github.com/bolderflight/MPU9250<br>

# romillion.ino




