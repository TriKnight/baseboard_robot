/*
@vAuthor: Tri Knight
@email: robotlab.vn@gmail.com
@rosserial library and transform the tf frame
*/
#include "ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>
//Loading config
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
// defines pins numbers of Back Ultrasonic Center
const int trigPin_BCenter = 7;
const int echoPin_BCenter = 6;
// defines pins numbers of Back Ultrasonic Left
const int trigPin_BLeft = 4;
const int echoPin_BLeft = 5;
// defines pins numbers of Back Ultrasonic Right
const int trigPin_BRight = 10;
const int echoPin_BRight = 11;


//Get IMU data
//----------------------------------
sensor_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("imu/data_raw", &raw_imu_msg);
//-----------------

void publishSonaBCenter()
{ // defines variables
    long duration;
    float distance;

    // Clears the trigPin
    digitalWrite(trigPin_BCenter, LOW);
    delayMicroseconds(2);

    // Sets the trigPin on HIGH state for 10 microseconds
    digitalWrite(trigPin_BCenter, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_BCenter, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin_BCenter, HIGH);

    // Calculating the distance (cm) = (duration [us] *air velocity[m/us]/2)
    distance= duration*0.034/2;
    //publishing data
    range_msg_1.range=distance;
    range_msg_1.header.stamp=nh.now();
    range_msg_1.header.frame_id = "sona_back_center";
    range_msg_1.header.seq = seq;
    seq = seq + 1;

    pub_range_back_center.publish(&range_msg_1);

}
void publishSonaBLeft()
{// defines variables
    long duration;
    float distance;

    // Clears the trigPin
    digitalWrite(trigPin_BLeft, LOW);
    delayMicroseconds(2);

    // Sets the trigPin on HIGH state for 10 microseconds
    digitalWrite(trigPin_BLeft, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_BLeft, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin_BLeft, HIGH);

    // Calculating the distance (cm) = (duration [us] *air velocity[m/us]/2)
    distance= duration*0.034/2;
    //publishing data
    range_msg_2.range=distance;
    range_msg_2.header.stamp=nh.now();
    range_msg_2.header.frame_id = "sona_back_left";
    range_msg_2.header.seq = seq;
    seq = seq + 1;

    pub_range_back_left.publish(&range_msg_2);

}

void publishSonaBRight()
{// defines variables
    long duration;
    float distance;

    // Clears the trigPin
    digitalWrite(trigPin_BRight, LOW);
    delayMicroseconds(2);

    // Sets the trigPin on HIGH state for 10 microseconds
    digitalWrite(trigPin_BRight, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_BRight, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin_BRight, HIGH);

    // Calculating the distance (cm) = (duration [us] *air velocity[m/us]/2)
    distance= duration*0.034/2;
    //publishing data
    range_msg_3.range=distance;
    range_msg_3.header.stamp=nh.now();
    range_msg_3.header.frame_id = "sona_back_right";
    range_msg_3.header.seq = seq;
    seq = seq + 1;

    pub_range_back_right.publish(&range_msg_3);

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
    pinMode(trigPin_BCenter, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin_BCenter, INPUT); // Sets the echoPin as an Input
    range_msg_1.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_1.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
    range_msg_1.min_range = 0.02; // Min detection object ~0.02 (m)
    range_msg_1.max_range = 4.5;  // max detection object ~4.5 (m)

      //Publish the Back ultrasonic left data
    nh.advertise(pub_range_back_left);
    pinMode(trigPin_BLeft, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin_BLeft, INPUT); // Sets the echoPin as an Input
    range_msg_2.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_2.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
    range_msg_2.min_range = 0.02; // Min detection object ~0.02 (m)
    range_msg_2.max_range = 4.5;  // max detection object ~4.5 (m)

      //Publish the Back ultrasonic right data
    nh.advertise(pub_range_back_right);
    pinMode(trigPin_BRight, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin_BRight, INPUT); // Sets the echoPin as an Input
    range_msg_3.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_3.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
    range_msg_3.min_range = 0.02; // Min detection object ~0.02 (m)
    range_msg_3.max_range = 4.5;  // max detection object ~4.5 (m)


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
    static unsigned long prev_sona_time_1=20;
    static unsigned long prev_sona_time_2=60;
    static unsigned long prev_sona_time_3=80;

    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;

    if ((millis() - prev_sona_time_3) >= (1000 / SONA_RATE_1))
    {
        publishSonaBRight();
        prev_sona_time_3 = millis();
    }

    if ((millis() - prev_sona_time_1) >= (1000 / SONA_RATE_2))
    {
        publishSonaBCenter();
        prev_sona_time_1 = millis();
    }

    if ((millis() - prev_sona_time_2) >= (1000 / SONA_RATE_3))
    {
        publishSonaBLeft();
        prev_sona_time_2 = millis();
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