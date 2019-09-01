# renew_turtlebot3
Using another board instead of OpenCr<br>
I use cortex-M4 board

# romillion_robot.h
This header file is to define your robot state

# romillion_core.h
There are a lot of header files, functions and variables

## f401reMap.h
**f401reMap.h** is the header file of cotex
Reference : https://www.instructables.com/id/Quick-Start-to-STM-Nucleo-on-Arduino-IDE/

## MPU9250.h
**MPU9250.h** is the header file of MPU9250
Reference : https://github.com/bolderflight/MPU9250<br>

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
# romillion.ino




