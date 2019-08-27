#ifndef ROMILLION_CORE_H_
#define ROMILLION_CORE_H_

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

#include <f401reMap.h>
#include "MPU9250.h"
#include "romillion_robot.h"

#include <math.h>

///////// TURTLEBOT3 //////////////////
#define FIRMWARE_VER "1.2.3"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

/////////////// MPU9250 ///////////////////////
#define sampleFreqDef   512.0f

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

void imu_init();
void updateMPU(void);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);

sensor_msgs::Imu getIMU(void);
sensor_msgs::MagneticField getMag(void);
sensor_msgs::Imu           imu_msg_;
sensor_msgs::MagneticField mag_msg_;

float gyro_X,gyro_Y,gyro_Z;
float acc_X,acc_Y,acc_Z;
float mag_X,mag_Y,mag_Z;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float deltat = 0.0f, sum = 0.0f;   
uint32_t lastUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval
uint32_t sumCount = 0;
static float com_ax=0,com_ay=0,com_az=0,com_gx=0,com_gy=0,com_gz=0;

float invSampleFreq = 1.0f / sampleFreqDef;
float beta = 0.1f;//sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta

/////////////// MOTOR ///////////////////////
#define cw_R 7
#define ccw_R 6
#define pwm_R 5
#define cw_L 11
#define ccw_L 10
#define pwm_L 9

//encoder interrupt pin
int encoder0PinA_L= pinMap(18);
int encoder0PinB_L = pinMap(19);
int encoder0PinA_R = pinMap(16);
int encoder0PinB_R = pinMap(17);

int encoder0Pos_R = 0,encoder0Pos_L = 0;
int prev_encoder_R=0, prev_encoder_L=0;
const float ratio =360./56./26.;//1456 pulse

int speed_R,speed_L;

bool ccw_R_dir,cw_R_dir;
bool ccw_L_dir,cw_L_dir;

/////////////// TURTLEBOT3 ///////////////////////

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void motorPowerCallback(const std_msgs::Bool& power_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

// Function prototypes
void publishImuMsg(void);
void publishMagMsg(void);
void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
void updateTFPrefix(bool isConnected);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);

void sendLogMsg(void);
void waitForSerialLink(bool isConnected);


/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

ros::Subscriber<std_msgs::Bool> motor_power_sub("motor_power", motorPowerCallback);

ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// Version information of Turtlebot3
turtlebot3_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("firmware_version", &version_info_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Odometry of Turtlebot3
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);

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
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;

#endif // TURTLEBOT3_CORE_CONFIG_H_
