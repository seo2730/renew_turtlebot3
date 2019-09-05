#include "romillion_core.h"
/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  Serial.begin(57600);
  motor_init(); // MOTOR SETTING
  imu_init();   // MPU9250 SETTING

  pinMode(button0,INPUT);
  pinMode(button1,INPUT);
  pinMode(button2,INPUT);

  digitalWrite(button0,HIGH);
  digitalWrite(button1,HIGH);
  digitalWrite(button2,HIGH);

  last_reading0 != digitalRead(button0);
  last_reading1 != digitalRead(button1);
  last_reading2 != digitalRead(button2);

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
  nh.subscribe(motor_power_sub);
  nh.subscribe(reset_sub);

  nh.advertise(sensor_state_pub);  
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(mag_pub);

  nh.advertise(sensor);
  nh.advertise(button); 

  tf_broadcaster.init(nh);

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  prev_update_time = millis();

  setup_end = true;
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  uint32_t t = millis();
  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

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

  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishSensorStateMsg();
    publishDriveInformation();
    tTime[2] = t;
  }
  
  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
    publishMagMsg();
    tTime[3] = t;
  }
  
  if ((t-tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishVersionInfoMsg();
    tTime[4] = t;
  }

#ifdef DEBUG
  if ((t-tTime[5]) >= (1000 / DEBUG_LOG_FREQUENCY))
  {
    sendDebuglog();
    tTime[5] = t;
  }
#endif

  //button
  push_button();
  
  // Send log message after ROS connection
  sendLogMsg();
  
  // Update the IMU unit
  updateMPU();
  
  // Start Gyro Calibration after ROS connection
  updateGyroCali(nh.connected());

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}

/*******************************************************************************
* BUTTON
*******************************************************************************/
void push_button()
{
  bool reading0=!digitalRead(button0),reading1=!digitalRead(button1),reading2=!digitalRead(button2);
       //reading3=!digitalRead(button3),reading4=!digitalRead(button4);

  if (last_reading0 != reading0 || last_reading1 != reading1 || last_reading2 != reading2 )//|| last_reading3 != reading3 || last_reading4 != reading4)
  {
      last_debounce_time = millis();
      published = false;
  }
  
  //if the button value has not changed for the debounce delay, we know its stable
  if (!published && (millis() - last_debounce_time)  > debounce_delay)
  {
      button_msg.button0=reading0;
      button_msg.button1=reading1;
      button_msg.button2=reading2;
      //button_msg.button3=reading3;
      //button_msg.button4=reading4;
      if(button_msg.button1==1)
      {
        switch(count)
        {
          case 0:analogWrite(pan_pwm,85);count++;break;
          case 1:analogWrite(pan_pwm,170);count++;break;
          case 2:analogWrite(pan_pwm,255);count++;break;
          case 3:analogWrite(pan_pwm,ran);count++;break;
          case 4:analogWrite(pan_pwm,0);count=0;break;
        }
      }
      button.publish(&button_msg);
      published = true;
  }
    if(count==4)
      analogWrite(pan_pwm,ran);
      
    last_reading0 = reading0;
    last_reading1 = reading1;
    last_reading2 = reading2;
    //last_reading3 = reading3;
    //last_reading4 = reading4;
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

  attachInterrupt(digitalPinToInterrupt(encoder0PinA_R), doEncoderA_R, CHANGE); // encoder pin on interrupt 0 (pin 2)
  
  //Left motor
  pinMode(cw_L,OUTPUT);
  pinMode(ccw_L,OUTPUT);
  pinMode(pwm_L,OUTPUT);
  pinMode(encoder0PinA_L, INPUT_PULLUP); 
  pinMode(encoder0PinB_L, INPUT_PULLUP); 
  
  attachInterrupt(digitalPinToInterrupt(encoder0PinA_L), doEncoderA_L, CHANGE); // encoder pin on interrupt 1 (pin 3)
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
  d_R=x*500+z*50;     
  d_L=x*500-z*50;
  speed_R=abs(x*500+z*50);     
  speed_L=abs(x*500-z*50);

  if(d_R>0 & d_L>0)
  {
    cw_R_dir= LOW;
    ccw_R_dir= HIGH;
    cw_L_dir= LOW; 
    ccw_L_dir= LOW;
  }

  else if(d_R>0 & d_L<0)
  {
    cw_R_dir= LOW;
    ccw_R_dir= HIGH;
    cw_L_dir= LOW; 
    ccw_L_dir= HIGH;
  }

  else if(d_R<0 & d_L>0)
  {
    cw_R_dir= LOW;
    ccw_R_dir= LOW;
    cw_L_dir= LOW; 
    ccw_L_dir= LOW;
  }

  else if(d_R<0 & d_L<0)
  {
    cw_R_dir= LOW;
    ccw_R_dir= LOW;
    cw_L_dir= LOW; 
    ccw_L_dir= HIGH;
  }

  else if(d_R==0 & d_L==0)
  {
    speed_R=0;
    speed_L=0;
  }
  
 speed_R=constrain(speed_R,0,255);
 speed_L=constrain(speed_L,0,255);
}

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

/*******************************************************************************
* MPU9250(필터 수정)
*******************************************************************************/
void imu_init(void)
{
  //start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-2000 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  // setting DLPF bandwidth to 184 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(0);
}

void updateMPU(void)
{
  IMU.readSensor();

  //상보필터
  
  gyro_X=0.95*IMU.getGyroX_rads()+0.05*com_gx;
  gyro_Y=0.95*IMU.getGyroY_rads()+0.05*com_gy;
  gyro_Z=0.95*IMU.getGyroZ_rads()+0.05*com_gz;

  acc_X=0.05*IMU.getAccelX_mss()+0.95*com_ax;
  acc_Y=0.05*IMU.getAccelY_mss()+0.95*com_ay;
  acc_Z=0.05*IMU.getAccelZ_mss()+0.95*com_az;

  com_gx=IMU.getGyroX_rads();
  com_gy=IMU.getGyroY_rads();
  com_gz=IMU.getGyroZ_rads();

  com_ax=IMU.getAccelX_mss();
  com_ay=IMU.getAccelY_mss();
  com_az=IMU.getAccelZ_mss();
  
  mag_X=IMU.getMagX_uT();
  mag_Y=IMU.getMagY_uT();
  mag_Z=IMU.getMagZ_uT();

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  MadgwickQuaternionUpdate(acc_X, acc_Y, acc_Z, gyro_X, gyro_Y, gyro_Z);
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];   // short name local variable for readability
  float norm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){ //return; // handle NaN
      norm = 1.0f/norm;
      ax *= norm;
      ay *= norm;
      az *= norm;
    
      // Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0;
      _4q1 = 4.0f * q1;
      _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1;
      _8q2 = 8.0f * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3;
    
      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
      norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
      s0 *= norm;
      s1 *= norm;
      s2 *= norm;
      s3 *= norm;
    
      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
  }

  // Integrate to yield quaternion
  q0 += qDot1 * deltat;//invSampleFreq;
  q1 += qDot2 * deltat;//invSampleFreq;
  q2 += qDot3 * deltat;//invSampleFreq;
  q3 += qDot4 * deltat;//invSampleFreq;
  norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q0 * norm;
  q[1] = q1 * norm;
  q[2] = q2 * norm;
  q[3] = q3 * norm;

}
void calibrationGyro()
{
  IMU.calibrateGyro();
}

sensor_msgs::Imu getIMU(void)
{
  imu_msg_.angular_velocity.x =  gyro_X;
  imu_msg_.angular_velocity.y = gyro_Y;
  imu_msg_.angular_velocity.z = gyro_Z;
  imu_msg_.angular_velocity_covariance[0] = 0.02;
  imu_msg_.angular_velocity_covariance[1] = 0;
  imu_msg_.angular_velocity_covariance[2] = 0;
  imu_msg_.angular_velocity_covariance[3] = 0;
  imu_msg_.angular_velocity_covariance[4] = 0.02;
  imu_msg_.angular_velocity_covariance[5] = 0;
  imu_msg_.angular_velocity_covariance[6] = 0;
  imu_msg_.angular_velocity_covariance[7] = 0;
  imu_msg_.angular_velocity_covariance[8] = 0.02;

  imu_msg_.linear_acceleration.x = acc_X;
  imu_msg_.linear_acceleration.y = acc_Y;
  imu_msg_.linear_acceleration.z = acc_Z;

  imu_msg_.linear_acceleration_covariance[0] = 0.04;
  imu_msg_.linear_acceleration_covariance[1] = 0;
  imu_msg_.linear_acceleration_covariance[2] = 0;
  imu_msg_.linear_acceleration_covariance[3] = 0;
  imu_msg_.linear_acceleration_covariance[4] = 0.04;
  imu_msg_.linear_acceleration_covariance[5] = 0;
  imu_msg_.linear_acceleration_covariance[6] = 0;
  imu_msg_.linear_acceleration_covariance[7] = 0;
  imu_msg_.linear_acceleration_covariance[8] = 0.04;

  imu_msg_.orientation.w = q[0];
  imu_msg_.orientation.x = q[1];
  imu_msg_.orientation.y = q[2];
  imu_msg_.orientation.z = q[3];

  imu_msg_.orientation_covariance[0] = 0.0025;
  imu_msg_.orientation_covariance[1] = 0;
  imu_msg_.orientation_covariance[2] = 0;
  imu_msg_.orientation_covariance[3] = 0;
  imu_msg_.orientation_covariance[4] = 0.0025;
  imu_msg_.orientation_covariance[5] = 0;
  imu_msg_.orientation_covariance[6] = 0;
  imu_msg_.orientation_covariance[7] = 0;
  imu_msg_.orientation_covariance[8] = 0.0025;

  return imu_msg_;
}

sensor_msgs::MagneticField getMag(void)
{
  mag_msg_.magnetic_field.x = mag_X;
  mag_msg_.magnetic_field.y = mag_Y;
  mag_msg_.magnetic_field.z = mag_Z;

  mag_msg_.magnetic_field_covariance[0] = 0.0048;
  mag_msg_.magnetic_field_covariance[1] = 0;
  mag_msg_.magnetic_field_covariance[2] = 0;
  mag_msg_.magnetic_field_covariance[3] = 0;
  mag_msg_.magnetic_field_covariance[4] = 0.0048;
  mag_msg_.magnetic_field_covariance[5] = 0;
  mag_msg_.magnetic_field_covariance[6] = 0;
  mag_msg_.magnetic_field_covariance[7] = 0;
  mag_msg_.magnetic_field_covariance[8] = 0.0048;

  return mag_msg_;
}

//////////////////////////// TURTLEBOT CODE ////////////////////////////////////

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  motor_speed(goal_velocity_from_cmd[LINEAR],goal_velocity_from_cmd[ANGULAR]);
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}

/*******************************************************************************
* Callback function for motor_power msg
*******************************************************************************/
void motorPowerCallback(const std_msgs::Bool& power_msg)
{
  bool dxl_power = power_msg.data;

//  motor_driver.setTorque(dxl_power);
}

/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{ 
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  calibrationGyro();
  
  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);  
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = getIMU();
  
  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  sensor_state_msg.header.stamp = rosNow();

  sensor_state_msg.left_encoder=encoder0Pos_L;
  sensor_state_msg.right_encoder=encoder0Pos_R;
  updateMotorInfo(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
  
  sensor_state_pub.publish(&sensor_state_msg);
}

/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
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

/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};
  
  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  //theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  theta = atan2f(q[1]*q[2] + q[0]*q[3],0.5f - q[2]*q[2] - q[3]*q[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity
  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  char log_msg[50];
  if (isConnected)
  {
    if (variable_flag == false)
    {      
      imu_init();
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{  
  goal_velocity[LINEAR]  =  goal_velocity_from_cmd[LINEAR];
  goal_velocity[ANGULAR] =  goal_velocity_from_cmd[ANGULAR];
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];  

  String name             = NAME;
  String firmware_version = FIRMWARE_VER;
  String bringup_log      = "This core(v" + firmware_version + ") is compatible with TB3 " + name;
   
  const char* init_log_data = bringup_log.c_str();

  if (nh.connected())
  {
    if (log_flag == false)
    {      
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
  }
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}
