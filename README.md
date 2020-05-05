# renew_turtlebot3
##### (Fail to optimization but it was a good experience to analyze turtlebot3 source)
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

## Motor control
void motor_speed(float x,float z) : x is linear z is angular<br>
d_R,d_L : <br>
>Choose direction(d_R>0,d_L>0:forward, d_R<0,d_L<0:backward, d_R>0,d_L<0:right, d_R<0,d_L>0:left)<br>

    /*******************************************************************************
    * Motor speed control
    *******************************************************************************/
    void motor_speed(float x,float z)
    {
      d_R=x*400+z*40;     
      d_L=x*400-z*40;
      speed_R=abs(x*400-z*40);     
      speed_L=abs(x*400+z*40);

      if(d_R>0 & d_L>0)
      {
        cw_L_dir= LOW;
        ccw_L_dir= HIGH;
        cw_R_dir= LOW; 
        ccw_R_dir= LOW;
      }

      else if(d_R>0 & d_L<0)
      {
        cw_L_dir= LOW;
        ccw_L_dir= HIGH;
        cw_R_dir= LOW; 
        ccw_R_dir= HIGH;
      }

      else if(d_R<0 & d_L>0)
      {
        cw_L_dir= LOW;
        ccw_L_dir= LOW;
        cw_R_dir= LOW; 
        ccw_R_dir= LOW;
      }

      else if(d_R<0 & d_L<0)
      {
        cw_L_dir= LOW;
        ccw_L_dir= LOW;
        cw_R_dir= LOW; 
        ccw_R_dir= HIGH;
      }

      else if(d_R==0 & d_L==0)
      {
        speed_R=0;
        speed_L=0;
      }

     speed_R=constrain(speed_R,0,255);
     speed_L=constrain(speed_L,0,255);
    }

Encoder : Calucate encoder<br>

    /*******************************************************************************
    * Encoder(전체적으로 확인한 뒤 보완할게 있으면 수정)
    *******************************************************************************/
    void doEncoderA_R(){
      if(goal_velocity[LINEAR]==0){
         encoder0Pos_R=prev_encoder_R;
      }

      else{
        if (digitalRead(encoder0PinA_R) == HIGH) { 
          // check channel B to see which way encoder is turning
          if (digitalRead(encoder0PinB_R) == LOW) {  
            encoder0Pos_R = encoder0Pos_R - 1;
          } 

          else {
            encoder0Pos_R = encoder0Pos_R + 1;
          }
        }

        else   // must be a high-to-low edge on channel A                                       
        { 
          // check channel B to see which way encoder is turning  
          if (digitalRead(encoder0PinB_R) == HIGH) {   
            encoder0Pos_R = encoder0Pos_R - 1;
          } 

          else {
            encoder0Pos_R = encoder0Pos_R + 1;
          }
        }
        prev_encoder_R=encoder0Pos_R;
      }
    }

    void doEncoderA_L(){
      if(goal_velocity[LINEAR]==0){
         encoder0Pos_L=prev_encoder_L;
      }

      else{
        if (digitalRead(encoder0PinA_L) == HIGH) { 
          // check channel B to see which way encoder is turning
          if (digitalRead(encoder0PinB_L) == LOW) {  
            encoder0Pos_L = encoder0Pos_L + 1;
          } 

          else {
            encoder0Pos_L = encoder0Pos_L - 1;
          }
        }

        else   // must be a high-to-low edge on channel A                                       
        { 
          // check channel B to see which way encoder is turning  
          if (digitalRead(encoder0PinB_L) == HIGH) {   
            encoder0Pos_L = encoder0Pos_L + 1;
          } 

          else {
            encoder0Pos_L = encoder0Pos_L - 1;
          }
        }
        prev_encoder_L=encoder0Pos_L;
      }
    }

The location of motor code in loop

    if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
    {
     updateGoalVelocity();
     if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT) {
      doMotor_R(cw_R_dir,ccw_R_dir,0);
      doMotor_L(cw_L_dir,ccw_L_dir,0);
     } else {
      doMotor_R(cw_R_dir,ccw_R_dir,speed_R);
      doMotor_L(cw_L_dir,ccw_L_dir,speed_L);
     }
     tTime[0] = t;
    }
