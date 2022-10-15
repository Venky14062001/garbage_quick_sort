#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#define FORCE_SENSOR_PIN A0 // the FSR and 10K pulldown are connected to A0
#define suction_pump 7

// defines variables
bool activate_suction = false;

//Publishers
std_msgs::Bool target_contact_msg;

ros::Publisher target_contact_pub("/garbage_quick_sort/arduino/target_contact", &target_contact_msg);

//Subsribers
void suctionCallback(const std_msgs::Bool& msg)
{
  activate_suction = msg.data;
}

//Callback
ros::Subscriber<std_msgs::Bool> suction_sub("/garbage_quick_sort/arduino/suction_state", &suctionCallback);
ros::NodeHandle nh;



bool force_reading(){
  int analogReading = analogRead(FORCE_SENSOR_PIN);
  Serial.println(analogReading);
  if (analogReading < 200)
  {
    return false;
  }

  else
  {
    return true;
  }
}
    
void setup() {
  pinMode(suction_pump, OUTPUT);
  Serial.begin(57600); // // Serial Communication is starting with 9600 of baudrate speed

  nh.initNode();
  nh.advertise(target_contact_pub);
  nh.subscribe(suction_sub);
}


void loop() {
  nh.spinOnce();
  force_reading();
  if(activate_suction){
    digitalWrite(suction_pump, HIGH);
  }
  else{
    digitalWrite(suction_pump, LOW);
  }
  
  target_contact_msg.data = force_reading();
  target_contact_pub.publish(&target_contact_msg);  
}
