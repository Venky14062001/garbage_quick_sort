#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#define distance_SENSOR_PIN A0 // the FSR and 10K pulldown are connected to A0
#define suction_pump 5 // change suction pump connection to pin 3 to avoid clash with active states
#define echoPin 2 // attach pin D4 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D5 Arduino to pin Trig of HC-SR04

// defines variables
bool suction_active = false;
long duration;
float distance; 

//Publishers
std_msgs::Float32 distance_msg;
ros::Publisher distance_pub("/garbage_quick_sort/arduino/distance", &distance_msg); 

ros::NodeHandle nh;

void suctionCallback(const std_msgs::Bool& msg)
{
  suction_active = msg.data;
}

//Callback
ros::Subscriber<std_msgs::Bool> suction_sub("/garbage_quick_sort/arduino/suction_active", &suctionCallback);

void distance_reading() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  distance_msg.data = distance;
}

void setup() 
{
  pinMode(suction_pump, OUTPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(57600); // // Serial Communication is starting with 9600 of baudrate speed

  nh.initNode();
  nh.advertise(distance_pub);
  nh.subscribe(suction_sub);
}

void loop() 
{
  nh.spinOnce();
  // if moving down and distance sensor detects, latch on the suction
  distance_reading();
  
  // keep publishing suction message for state machine to change state!
  if (suction_active)
  {
    digitalWrite(suction_pump, LOW);
  }

  else
  {
    digitalWrite(suction_pump, HIGH);
  }
 
  distance_pub.publish(&distance_msg);
  delay(10);
}
