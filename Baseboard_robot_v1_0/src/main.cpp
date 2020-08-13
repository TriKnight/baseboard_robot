/*
@vAuthor: Tri Knight
@email: robotlab.vn@gmail.com
@rosserial library and transform the tf frame
*/
#include "ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>
//Loading config

//Include the Newping Ultrasonic Sensor
#include "NewPing.h"
#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
// defines pins numbers of Back Ultrasonic Center
const int trigPin_BCenter = 7;
const int echoPin_BCenter = 6;
// defines pins numbers of Back Ultrasonic Left
const int trigPin_BLeft = 4;
const int echoPin_BLeft = 5;
// defines pins numbers of Back Ultrasonic Right
const int trigPin_BRight = 10;
const int echoPin_BRight = 11;

//-----------------------------------------
#include "robotlab_base_config.h"
// include ROS lib for Range sensor
#include <sensor_msgs/Range.h>
// include ROS lib for publishing IMU
#include "sensor_msgs/Imu.h"
//Define hardware typeIMU
#include "Imu.h"
#define IMU_PUBLISH_RATE 20 //hz
#define SONA_RATE_1 40 //hz
#define SONA_RATE_2 35 //hz
#define SONA_RATE_3 30 //hz
#define DEBUG_RATE 5


int32_t seq;
//ROS node handle
ros::NodeHandle nh;

//Get Range sensor data
//----------------------------------
sensor_msgs::Range range_msg_1;
sensor_msgs::Range range_msg_2;
sensor_msgs::Range range_msg_3;
ros::Publisher pub_range_back_center("sona_back_center", &range_msg_1);
ros::Publisher pub_range_back_left("sona_back_left", &range_msg_2);
ros::Publisher pub_range_back_right("sona_back_right", &range_msg_3);
NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(trigPin_BCenter, echoPin_BCenter, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(trigPin_BLeft, echoPin_BLeft, MAX_DISTANCE), 
  NewPing(trigPin_BRight, echoPin_BRight, MAX_DISTANCE)
};


//Get IMU data
//----------------------------------
sensor_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("imu/data_raw", &raw_imu_msg);
//-----------------

void publishSonaBack()
{ // defines variables
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
   
    float distance_1;
    float distance_2;
    float distance_3;

    distance_1=sonar[0].ping_cm();
    range_msg_1.range=distance_1;
    range_msg_1.header.stamp=nh.now();
    range_msg_1.header.frame_id = "sona_back_center";
    range_msg_1.header.seq = seq;
    
    distance_2=sonar[1].ping_cm();
    range_msg_2.range=distance_2;
    range_msg_2.header.stamp=nh.now();
    range_msg_2.header.frame_id = "sona_back_left";
    range_msg_2.header.seq = seq;

    distance_3=sonar[2].ping_cm();
    range_msg_3.range=distance_3;
    range_msg_3.header.stamp=nh.now();
    range_msg_3.header.frame_id = "sona_back_right";
    range_msg_3.header.seq = seq;

    seq = seq + 1;
    pub_range_back_center.publish(&range_msg_1);
    pub_range_back_left.publish(&range_msg_2);
    pub_range_back_right.publish(&range_msg_3);
    
  }

    // Calculating the distance (cm) = (duration [us] *air velocity[m/us]/2)
    
    //publishing data
    

}

void publishIMU()
{

    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object
    raw_imu_msg.header.stamp=nh.now();
    raw_imu_msg.header.frame_id = "imu_link";
    raw_imu_msg.header.seq = seq;
    seq = seq + 1;

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}
void printDebug()
{

}
//Setup data
void setup()
{
    //Initial ROS node and set baudrate
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    //Publish the IMU data
    nh.advertise(raw_imu_pub);
    //Publish the Back ultrasonic center data
    nh.advertise(pub_range_back_center);

    range_msg_1.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_1.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
    range_msg_1.min_range = 0.02; // Min detection object ~0.02 (m)
    range_msg_1.max_range = 2.0;  // max detection object ~4.5 (m)

      //Publish the Back ultrasonic left data
    nh.advertise(pub_range_back_left);

    range_msg_2.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_2.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
    range_msg_2.min_range = 0.02; // Min detection object ~0.02 (m)
    range_msg_2.max_range = 2.0;  // max detection object ~4.5 (m)

      //Publish the Back ultrasonic right data
    nh.advertise(pub_range_back_right);

    range_msg_3.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_3.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
    range_msg_3.min_range = 0.02; // Min detection object ~0.02 (m)
    range_msg_3.max_range = 2.0;  // max detection object ~4.5 (m)


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
    static unsigned long prev_sona_time=20;


    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;

    if ((millis() - prev_sona_time) >= (1000 / SONA_RATE_1))
    {
        publishSonaBack();
        prev_sona_time = millis();
    }



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