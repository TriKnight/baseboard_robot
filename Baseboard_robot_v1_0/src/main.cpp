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
#define SONAR_NUM 4     // Number of sensors.
#define MAX_DISTANCE 220 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Looping the ping as 60 ms (must be try and error number >=60ms)

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.


#define SONA_RATE 10  //hz
#define SAFETY_RATE 5      //hz
#define DEBUG_RATE 1

// Define sequence of the header
int32_t seq;
//ROS node handle
ros::NodeHandle nh;
//-------------------------------------------
                       // defines Ultrasonic paramter

 //Store filtered sensor's value.
uint8_t leftFSensor;
uint8_t rightFSensor;
uint8_t leftBSensor;
uint8_t rightBSensor;
uint8_t rangeSensor[] = {leftFSensor, rightFSensor, leftBSensor, rightBSensor}; //define

uint8_t leftFKalman; //Store filtered sensor's value.
uint8_t rightFKalman;
uint8_t leftBKalman;
uint8_t rightBKalman;
//-----------------------------------------
// 3. Get sensor data in ROS message
//-----------------------------------------
//Get Range sensor data
//----------------------------------
sensor_msgs::Range range_msg_1;
sensor_msgs::Range range_msg_2;
sensor_msgs::Range range_msg_3;
sensor_msgs::Range range_msg_4;
ros::Publisher pub_range_front_left("sona_front_left", &range_msg_1);
ros::Publisher pub_range_front_right("sona_front_right", &range_msg_2);
ros::Publisher pub_range_back_left("sona_back_left", &range_msg_3);
ros::Publisher pub_range_back_right("sona_back_right", &range_msg_4);

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(4, 5, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(6, 7, MAX_DISTANCE),
  NewPing(10, 11, MAX_DISTANCE),
  NewPing(24, 25, MAX_DISTANCE)
  
};


//-------- Kalman Filter ------------
SimpleKalmanFilter KF_FLeft(2, 2, 0.1);
SimpleKalmanFilter KF_FRight(2, 2, 0.1);
SimpleKalmanFilter KF_BLeft(2, 2, 0.1);
SimpleKalmanFilter KF_BRight(2, 2, 0.1);

void applyKF(uint8_t leftFSensor, uint8_t rightFSensor, uint8_t leftBSensor, uint8_t rightBSensor){
    leftFKalman = KF_FLeft.updateEstimate(leftFSensor);
    rightFKalman = KF_FRight.updateEstimate(rightFSensor);
    leftBKalman = KF_BLeft.updateEstimate(leftBSensor);
    rightBKalman = KF_BRight.updateEstimate(rightBSensor);
}

void setup() {
  Serial.begin(115200);
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  //Initial ROS node and set baudrate
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  //Publish the Back ultrasonic center data
  nh.advertise(pub_range_front_left);
  range_msg_1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_1.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
  range_msg_1.min_range = 0.02;     // Min detection object ~0.02 (m)
  range_msg_1.max_range = 2.2;      // max detection object ~4.5 (m)
  
  nh.advertise(pub_range_front_right);
  range_msg_2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_2.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
  range_msg_2.min_range = 0.02;     // Min detection object ~0.02 (m)
  range_msg_2.max_range = 2.2;      // max detection object ~4.5 (m)

  nh.advertise(pub_range_back_left);
  range_msg_3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_3.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
  range_msg_3.min_range = 0.02;     // Min detection object ~0.02 (m)
  range_msg_3.max_range = 2.2;      // max detection object ~4.5 (m)


  nh.advertise(pub_range_back_right);
  range_msg_4.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_4.field_of_view = 0.26; // FOV of the Ultrasound = 0.26 (rad) ~ 15 (deg)
  range_msg_4.min_range = 0.02;     // Min detection object ~0.02 (m)
  range_msg_4.max_range = 2.2;      // max detection object ~4.5 (m)

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("ROBOTLAB CONNECTED");
    delay(1);

}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
    static unsigned long prev_sona_time = 0;
    //this block publishes the Sona
    if ((millis() - prev_sona_time) >= (1000 / SONA_RATE))
    {
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    rangeSensor[i] = cm[i];
  }
    applyKF(rangeSensor[0], rangeSensor[1], rangeSensor[2], rangeSensor[3]);
    range_msg_1.range = leftFKalman;
    range_msg_2.range = rightFKalman;
    range_msg_3.range = leftBKalman;
    range_msg_4.range = rightBKalman;

    range_msg_1.header.stamp = nh.now();
    range_msg_2.header.stamp = nh.now();
    range_msg_3.header.stamp = nh.now();
    range_msg_4.header.stamp = nh.now();

    range_msg_1.header.frame_id = "sona_front_left";
    range_msg_1.header.seq = seq;
    seq = seq + 1;
    range_msg_2.header.frame_id = "sona_front_right";
    range_msg_2.header.seq = seq;
    seq = seq + 1;
    range_msg_3.header.frame_id = "sona_back_left";
    range_msg_3.header.seq = seq;
    seq = seq + 1;
    range_msg_4.header.frame_id = "sona_back_right";
    range_msg_4.header.seq = seq;
    seq = seq + 1;

    pub_range_front_left.publish(&range_msg_1);
    pub_range_front_right.publish(&range_msg_2);
    pub_range_back_left.publish(&range_msg_3);
    pub_range_back_right.publish(&range_msg_4);
    
    prev_sona_time = millis(); //set prev_sona_time
}
}
void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
   nh.spinOnce();
  // Other code that *DOESN'T* analyze ping results can go here.
}
