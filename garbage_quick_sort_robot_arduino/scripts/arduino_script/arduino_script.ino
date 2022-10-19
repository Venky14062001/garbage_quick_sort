#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <garbage_quick_sort_robot_msg/RobotState.h>

// subscribe to robot global state, if correct state, then keep monitoring force sensor, as soon as force detected turn on suction and reset force sensor
// suction remains turned on until state reaches drop location, then suction releases and everything resets

#define FORCE_SENSOR_PIN A0 // the FSR and 10K pulldown are connected to A0
#define suction_pump 3 // change suction pump connection to pin 3 to avoid clash with active states



// defines variables
bool force_sensed = false;
bool suction_active = false;
garbage_quick_sort_robot_msg::RobotState current_global_state;
int suction_active_states[7] = {5, 6, 7, 8, 9, 10, 11};

//Publishers
std_msgs::Bool suction_msg;
std_msgs::Int16 force_msg;

ros::Publisher suction_pub("/garbage_quick_sort/arduino/suction_active", &suction_msg); 
ros::Publisher force_pub("/garbage_quick_sort/arduino/force", &force_msg); 

ros::NodeHandle nh;


void globalStateCallback(const garbage_quick_sort_robot_msg::RobotState& msg)
{
  current_global_state = msg;
  if ((current_global_state.robot_state == suction_active_states[0]) || (current_global_state.robot_state == suction_active_states[1]) ||
            (current_global_state.robot_state == suction_active_states[2]) || (current_global_state.robot_state == suction_active_states[3]) ||
            (current_global_state.robot_state == suction_active_states[4]) || (current_global_state.robot_state == suction_active_states[5])
            || (current_global_state.robot_state == suction_active_states[6])){
              nh.loginfo("Arduino State");

            }

}


//Callback
ros::Subscriber<garbage_quick_sort_robot_msg::RobotState> global_state_sub("/garbage_quick_sort/global_state", &globalStateCallback);

bool force_reading()
{
  int analogReading = analogRead(FORCE_SENSOR_PIN);
  force_msg.data = analogReading;
//  Serial.println(analogReading);
  if (analogReading > 100)
  {
    return true;
  }

  else
  {
    return false;
  }
}
    
void setup() 
{
  pinMode(suction_pump, OUTPUT);
  Serial.begin(57600); // // Serial Communication is starting with 9600 of baudrate speed

  nh.initNode();
  nh.advertise(suction_pub);
  nh.advertise(force_pub);
  nh.subscribe(global_state_sub);
}

void loop() 
{
//      digitalWrite(suction_pump, LOW);
//      Serial.println("active");
//      delay(2000);
//      
////      digitalWrite(suction_pump, HIGH);
////      Serial.println("no");
////          delay(2000);


  nh.spinOnce();
  // if moving down and force sensor detects, latch on the suction
  force_reading();
  if ((current_global_state.robot_state == suction_active_states[0]) && (!suction_active))
  {
    //suction_active = force_reading();
    nh.loginfo("set suction true");
    suction_active = true;
    //if (suction_active) {delay(500);}
  }
  
  // if in any of the other active states and suction was latched high, keep it latched high
  else if ((current_global_state.robot_state == suction_active_states[0]) || (current_global_state.robot_state == suction_active_states[1]) ||
            (current_global_state.robot_state == suction_active_states[2]) || (current_global_state.robot_state == suction_active_states[3]) ||
            (current_global_state.robot_state == suction_active_states[4]) || (current_global_state.robot_state == suction_active_states[5])
            || (current_global_state.robot_state == suction_active_states[6]))
  {
    if(suction_active)
    {
      digitalWrite(suction_pump, LOW);
      nh.loginfo("active");
    }

//    else
//    {
//      digitalWrite(suction_pump, HIGH);
//      //reset suction_active
//      nh.loginfo("no");
//
//      if (suction_active) suction_active = false;
//    }
  }
    
  // if not in active state, then suction LOW
  else
  {
    digitalWrite(suction_pump, HIGH);
    //reset suction_active
      nh.loginfo("no2");

    if (suction_active) suction_active = false;
  }
  
  // keep publishing suction message for state machine to change state!

  suction_msg.data = suction_active;
  suction_pub.publish(&suction_msg);  
  force_pub.publish(&force_msg);
  delay(10);
}
