/* 
 * rosserial IR Ranger Example  
 * 
 * This example is calibrated for the Sharp GP2D120XJ00F.
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;


sensor_msgs::Range range_msg_0;
ros::Publisher pub_range_0( "cwru/range0", &range_msg_0);
sensor_msgs::Range range_msg_1;
ros::Publisher pub_range_1( "cwru/range1", &range_msg_1);

const int analog_pin_0 = 0;
const int analog_pin_1 = 0;
unsigned long range_timer;

int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

/*
 * getRange() - samples the analog input from the ranger
 * and converts it into meters.  
 */
float getRange(int pin_num){
    int sample;
    // Get data
    sample = averageAnalog(pin_num);
    return sample;
    // if the ADC reading is too low, 
    //   then we are really far away from anything
    if(sample < 10)
        return 254;     // max range
    // Magic numbers to get cm
    sample= 1309/(sample-3);
    return (sample - 1)/100; //convert to meters
}

char frameid0[] = "/ir_ranger0";
char frameid1[] = "/ir_ranger1";

void setup()
{
  nh.initNode();
  nh.advertise(pub_range_0);
  nh.advertise(pub_range_1);
  
  range_msg_0.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg_0.header.frame_id =  frameid0;
  range_msg_0.field_of_view = 0.01;
  range_msg_0.min_range = 0.03;
  range_msg_0.max_range = 0.4;
  
  range_msg_1.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg_1.header.frame_id =  frameid1;
  range_msg_1.field_of_view = 0.01;
  range_msg_1.min_range = 0.03;
  range_msg_1.max_range = 0.4;
  
}

void loop()
{
  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > 100){
    ros::Time t = nh.now();
    range_msg_0.range = getRange(analog_pin_0);
    range_msg_0.header.stamp = t;
    range_msg_1.range = getRange(analog_pin_1);
    range_msg_1.header.stamp = t;
    
    pub_range_0.publish(&range_msg_0);
    pub_range_1.publish(&range_msg_1);
    range_timer =  millis();
  }
  nh.spinOnce();
}

