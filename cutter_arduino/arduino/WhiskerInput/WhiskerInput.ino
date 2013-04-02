/* 
 * rosserial Whisker Input for CWRU Cutter  
 * 
 * returns Analog readings from ADC potentiometers
 *
 * Note that in order to get the Arduino Compiler to find the 
 * ROS messages, you need to run the following command line
 * program:
 *   rosrun rosserial_client make_library.py path_to_libraries message_package
 *  for example:
 *   "rosrun rosserial_client ~/Desktop/arduino-1.0.2/libraries cutter_arduino"
 */

#include <ros.h>
#include <ros/time.h>
//#include <sensor_msgs/Range.h>
//#include <cutter_arduino/Analog_Rangefinder.h>
#include <cutter_arduino/Potentiometer.h>

ros::NodeHandle  nh;

cutter_arduino::Potentiometer pot_msg;
ros::Publisher pub_whiskers( "cwru/whiskers", &pot_msg);

const int analog_pin_0 = 0;
const int analog_pin_1 = 1;
unsigned long range_timer;
unsigned long new_time;

int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

void setup()
{
  nh.initNode();
  nh.advertise(pub_whiskers);  
}

void loop()
{
  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  new_time = millis();
  if ( (new_time-range_timer) > 50){
    range_timer = new_time;
    
    pot_msg.pot0 = averageAnalog(analog_pin_0);
    pot_msg.pot1 = averageAnalog(analog_pin_1);
    
    pub_whiskers.publish(&pot_msg);
  }
  nh.spinOnce();
}

