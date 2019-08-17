#include "romillion_core.h"



void setup() {
  Serial.begin(57600);

  motor_init();
  
  nh.initNode();
  nh.subscribe(cmd_vel_sub);

  nh.advertise(cmd_vel_rc100_pub);
}

void loop() {
  updateGoalVelocity();
  
  nh.spinOnce();
  
}
/*******************************************************************************
* Motor init
*******************************************************************************/
void motor_init()
{
  //Right motor
  pinMode(cw_R,OUTPUT);
  pinMode(ccw_R,OUTPUT);
  pinMode(pwm_R,OUTPUT);
  pinMode(encoder0PinA_R, INPUT_PULLUP); 
  pinMode(encoder0PinB_R, INPUT_PULLUP); 

  attachInterrupt(0, doEncoderA_R, CHANGE); // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(2, doEncoderB_R, CHANGE); // encoder pin on interrupt 0 (pin 2)
  
  //Left motor
  pinMode(cw_L,OUTPUT);
  pinMode(ccw_L,OUTPUT);
  pinMode(pwm_L,OUTPUT);
  pinMode(encoder0PinA_L, INPUT_PULLUP); 
  pinMode(encoder0PinB_L, INPUT_PULLUP); 
  
  attachInterrupt(1, doEncoderA_L, CHANGE); // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(3, doEncoderB_R, CHANGE); // encoder pin on interrupt 0 (pin 2)
}

void doMotor_R(bool dir1,bool dir2,int vel){
  digitalWrite(cw_R, dir1);
  digitalWrite(ccw_R, dir2); 
  analogWrite(pwm_R,vel);
}

void doMotor_L(bool dir1,bool dir2,int vel){
  digitalWrite(cw_L, dir1);
  digitalWrite(ccw_L, dir2); 
  analogWrite(pwm_L,vel);
}

/*******************************************************************************
* Motor speed control
*******************************************************************************/
void motor_speed(float x,float z)
{
  if(x>=0){
      cw_R_dir= LOW;
      ccw_R_dir= HIGH;
      cw_L_dir= LOW; 
      ccw_L_dir= LOW;
      speed_R=x*1000+z*100;     
      speed_L=x*1000-z*100; 
  }
  else{
      cw_R_dir= LOW;
      ccw_R_dir= LOW;
      cw_L_dir= LOW;
      ccw_L_dir= HIGH;      
      speed_R=-x*1000+z*100;     
      speed_L=-x*1000-z*100;
  }

  if(x==0){
    if(z>0){
      cw_R_dir= LOW;
      ccw_R_dir= HIGH;
      cw_L_dir= LOW; 
      ccw_L_dir= HIGH;
      speed_R=z*100;     
      speed_L=z*100; 
    }
    else{
      cw_R_dir= LOW;
      ccw_R_dir= LOW;
      cw_L_dir= LOW; 
      ccw_L_dir= LOW;
      speed_R=-z*100;     
      speed_L=-z*100; 
    }
  }

  speed_R=constrain(speed_R,0,255);
  speed_L=constrain(speed_L,0,255);
}

/*******************************************************************************
* Encoder
*******************************************************************************/
void doEncoderA_R(){
  // look for a low-to-high on channel A
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
}

void doEncoderB_R(){
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinB_R) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA_R) == LOW) {  
      encoder0Pos_R = encoder0Pos_R - 1;
    } 

    else {
      encoder0Pos_R = encoder0Pos_R + 1;
    }
  }

  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA_R) == HIGH) {   
      encoder0Pos_R = encoder0Pos_R - 1;
    } 

    else {
      encoder0Pos_R = encoder0Pos_R + 1;
    }
  }
}

void doEncoderA_L(){
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA_L) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB_L) == LOW) {  
      encoder0Pos_L = encoder0Pos_L - 1;
    } 

    else {
      encoder0Pos_L = encoder0Pos_L + 1;
    }
  }

  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB_L) == HIGH) {   
      encoder0Pos_L = encoder0Pos_L - 1;
    } 

    else {
      encoder0Pos_L = encoder0Pos_L + 1;
    }
  }
}

void doEncoderB_L(){
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinB_L) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA_L) == LOW) {  
      encoder0Pos_L = encoder0Pos_L - 1;
    } 

    else {
      encoder0Pos_L = encoder0Pos_L + 1;
    }
  }

  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA_L) == HIGH) {   
      encoder0Pos_L = encoder0Pos_L - 1;
    } 

    else {
      encoder0Pos_L = encoder0Pos_L + 1;
    }
  }
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  motor_speed(goal_velocity_from_cmd[LINEAR],goal_velocity_from_cmd[ANGULAR]);
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  //goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  //goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
  
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  doMotor_R(cw_R_dir,ccw_R_dir,speed_R);
  doMotor_L(cw_L_dir,ccw_L_dir,speed_L);
  goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_rc100[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];

  //sensors.setLedPattern(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}
