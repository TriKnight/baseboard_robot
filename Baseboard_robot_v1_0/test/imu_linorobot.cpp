#include "ros.h"
#include "ros/time.h"
//Loading config
#include "robotlab_base_config.h"
//header file for publishing IMU
#include "sensor_msgs/Imu.h"

#include "Imu.h"

#define IMU_PUBLISH_RATE 20 //hz
#define DEBUG_RATE 5

//ROS node handle
ros::NodeHandle nh;

//Get IMU data
sensor_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("imu/data_raw", &raw_imu_msg);

//-----------------

void publishIMU()
{

    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object


    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}
void printDebug()
{

}
//Setup data
void setup()
{

    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.advertise(raw_imu_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("ROBOTLAB CONNECTED");
    delay(1);
}
// Loop data
void loop()
{
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;


    //this block publishes the IMU data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if (imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            publishIMU();
        }
        prev_imu_time = millis();
    }

    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if (DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            //printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}