#ifndef ROMILLION_CORE__H_
#define ROMILLION_CORE__H_

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>

//내가 만든 msg 파일 
#include <arduino_ros/arduino.h>
#include <arduino_ros/button.h>

#include "romillion_robot.h"

#include <math.h>

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

///////////// MOTOR /////////////////
#define encoder0PinA_R 2
#define encoder0PinB_R 21
#define cw_R 7
#define ccw_R 6
#define pwm_R 5

#define encoder0PinA_L 3
#define encoder0PinB_L 20
#define cw_L 10
#define ccw_L 9
#define pwm_L 8


void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void updateGoalVelocity(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateTFPrefix(bool isConnected);

/*******************************************************************************
* MOTOR
*******************************************************************************/
int n;
int encoder0Pos_R = 0;
int encoder0Pos_L = 0;
const float ratio =360./56./26.;//1456 pulse

float speed_R;
float speed_L;

static int count_R=0;
static int count_L=0;

bool ccw_R_dir;
bool cw_R_dir;

bool ccw_L_dir;
bool cw_L_dir;

bool work=LOW;

int cur_speed_R = 0;
int cur_speed_L = 0;
int old_encoder0Pos_R = 0;
int old_encoder0Pos_L = 0;
int spd_cnt =0;
int torque_cnt =0;
int err_spd_R =0;
int err_spd_L =0;
int loop_cnt = 0;
int old_speed_R =0;
int old_speed_L =0;
bool R_state = LOW;
bool L_state = LOW;

int tar_spd_R=0;
int tar_spd_L=0;

int count=0;
char c='0';

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/


/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float previ_goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};







#endif
