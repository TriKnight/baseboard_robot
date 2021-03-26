/*
@vAuthor: Tri Knight
@email: robotlab.vn@gmail.com
@rosserial library and transform the tf frame
*/
//-----------------------------------------
// 1. Include all libraries
//-----------------------------------------
#include "ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>
#include "SimpleKalmanFilter.h"
#include "std_msgs/String.h"
#include "robotlab_base_config.h"
// include ROS lib for Range sensor
#include <sensor_msgs/Range.h>
// include ROS lib for publishing IMU
#include "sensor_msgs/Imu.h"
//Define hardware typeIMU
#include "Imu.h" // Include the IMU sensor
#include "NewPing.h" // Include the ultrasonics sensor

//-----------------------------------------
// 2. Define all parameter
//-----------------------------------------
// Ultrasonic pararmeter
#define SONAR_NUM 3     // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 50 // Looping the ping as 50 ms
unsigned long pingTimer= 0;
uint8_t currentSensor = 0;     // Keeps track of which sensor is active.

// defines pins numbers of Back Ultrasonic Center
const int trigPin_BCenter = 47;
const int echoPin_BCenter = 48;
// defines pins numbers of Back Ultrasonic Left
const int trigPin_BLeft = 2;
const int echoPin_BLeft = 3;
// defines pins numbers of Back Ultrasonic Right
const int trigPin_BRight = 10;
const int echoPin_BRight = 11;



#define SONA_RATE 20  //hz
#define SAFETY_RATE 5      //hz
#define DEBUG_RATE 1

// Define sequence of the header
int32_t seq;
//ROS node handle
ros::NodeHandle nh;
//-------------------------------------------
                       // defines Ultrasonic paramter

 //Store filtered sensor's value.
uint8_t leftBSensor;
uint8_t centerBSensor;
uint8_t rightBSensor;
uint8_t rangeSensor[] = {leftBSensor,centerBSensor,rightBSensor}; //define
uint8_t leftSensorKalman; //Store filtered sensor's value.
uint8_t centerSensorKalman;
uint8_t rightSensorKalman;
//-----------------------------------------
// 3. Get sensor data in ROS message
//-----------------------------------------
//Get Range sensor data
//----------------------------------
sensor_msgs::Range range_msg_1;
sensor_msgs::Range range_msg_2;
sensor_msgs::Range range_msg_3;
ros::Publisher pub_range_back_center("sona_back_center", &range_msg_1);
ros::Publisher pub_range_back_left("sona_back_left", &range_msg_2);
ros::Publisher pub_range_back_right("sona_back_right", &range_msg_3);
NewPing sonar[SONAR_NUM] = {                                 // Sensor object array.
    NewPing(trigPin_BCenter, echoPin_BCenter, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
    NewPing(trigPin_BLeft, echoPin_BLeft, MAX_DISTANCE),
    NewPing(trigPin_BRight, echoPin_BRight, MAX_DISTANCE)
    };

//-----------------
/*
  create Kalman filter objects for the sensors.
   SimpleKalmanFilter(e_mea, e_est, q);
   e_mea: Measurement Uncertainty
   e_est: Estimation Uncertainty
   q: Process Noise
*/
//--------- Safety Feature -----------
std_msgs::String safety_msg;
ros::Publisher safety("safety_sona", &safety_msg);
char safety_activate[16] = "safety_activate";
char safety_disactivate[19] = "safety_disactivate";

//-------- Kalman Filter ------------
SimpleKalmanFilter KF_Left(2, 2, 0.1);
SimpleKalmanFilter KF_Center(2, 2, 0.1);
SimpleKalmanFilter KF_Right(2, 2, 0.1);

void safetyActivate() // sona_range and safety_range in cm
{
    digitalWrite(8, HIGH); // turn the LED ON
    safety_msg.data = safety_activate;
    safety.publish(&safety_msg);
}

void safetyDisactivate() // sona_range and safety_range in cm
{
    digitalWrite(8, LOW); // turn the LED OFF
    safety_msg.data = safety_disactivate;
    safety.publish(&safety_msg);
}

void applyKF(){
    leftSensorKalman = KF_Left.updateEstimate(leftBSensor);
    centerSensorKalman = KF_Center.updateEstimate(centerBSensor);
    rightSensorKalman = KF_Right.updateEstimate(rightBSensor);
}

void sensorArray(){
    leftBSensor = rangeSensor[0];
    centerBSensor = rangeSensor[1];
    rightBSensor = rangeSensor[2];
    
}
void getSona()
{   
    //Average ultrasonic
    static unsigned long prev_time = 0;
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    { // Loop through each sensor and display results.
       
        if((millis() -prev_time) >= pingTimer){
            pingTimer += PING_INTERVAL * i;
            rangeSensor[i]=sonar[i].ping_cm();
            //sonar[currentSensor].timer_stop();
            prev_time = millis();
        }
    }
    sensorArray();
    pingTimer =0;
    prev_time = 0;
    // Calculating the distance (cm) = (duration [us] *air velocity[m/us]/2)

    //publishing data

}
void publishSona(uint8_t leftSensorKalman,uint8_t centerSensorKalman, uint8_t rightSensorKalman){
    range_msg_1.range = centerSensorKalman;
    range_msg_2.range = leftSensorKalman;
    range_msg_3.range = rightSensorKalman;
    

    range_msg_1.header.stamp = nh.now();
    range_msg_2.header.stamp = nh.now();
    range_msg_3.header.stamp = nh.now();

    range_msg_1.header.frame_id = "sona_back_center";
    range_msg_1.header.seq = seq;
    seq = seq + 1;
    range_msg_2.header.frame_id = "sona_back_left";
    range_msg_2.header.seq = seq;
    seq = seq + 1;
    range_msg_3.header.frame_id = "sona_back_right";
    range_msg_3.header.seq = seq;
    seq = seq + 1;

    pub_range_back_center.publish(&range_msg_1);
    pub_range_back_left.publish(&range_msg_2);
    pub_range_back_right.publish(&range_msg_3);
}

void printDebug()
{
};
//Setup data
void setup()
{
    // Setting harware
    pinMode(8, OUTPUT); // Red LED Enable

    //Initial ROS node and set baudrate
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    //Publish the Safety Message
    nh.advertise(safety);

    //Publish the Back ultrasonic center data
    nh.advertise(pub_range_back_center);

    range_msg_1.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_1.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
    range_msg_1.min_range = 0.02;     // Min detection object ~0.02 (m)
    range_msg_1.max_range = 2.0;      // max detection object ~4.5 (m)

    //Publish the Back ultrasonic left data
    nh.advertise(pub_range_back_left);

    range_msg_2.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_2.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
    range_msg_2.min_range = 0.02;     // Min detection object ~0.02 (m)
    range_msg_2.max_range = 2.0;      // max detection object ~4.5 (m)

    //Publish the Back ultrasonic right data
  nh.advertise(pub_range_back_right);

  range_msg_3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_3.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
  range_msg_3.min_range = 0.02;     // Min detection object ~0.02 (m)
   range_msg_3.max_range = 2.0;      // max detection object ~4.5 (m)

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
    static unsigned long prev_sona_time = 0;
    static unsigned long prev_debug_time = 0;
    //this block publishes the Sona
    if ((millis() - prev_sona_time) >= (1000 / SONA_RATE))
    {
        getSona();
        applyKF();
        publishSona(leftSensorKalman, centerSensorKalman, rightSensorKalman); // Publish Sona data
        prev_sona_time = millis(); //set prev_sona_time
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