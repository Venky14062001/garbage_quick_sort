#include <ros.h>
#include <std_msgs/Bool.h>
#include <garbage_quick_sort_robot_msg/RobotState.h>

// subscribe to robot global state, if correct state, then keep monitoring force sensor, as soon as force detected turn on suction and reset force sensor
// suction remains turned on until state reaches drop location, then suction releases and everything resets

#define FORCE_SENSOR_PIN A0 // the FSR and 10K pulldown are connected to A0
#define suction_pump 3 // change suction pump connection to pin 3 to avoid clash with active states

// defines variables
bool force_sensed = false;
bool suction_active = false;
garbage_quick_sort_arduino::RobotState current_global_state;
int suction_active_states[5] = {4, 5, 6, 7, 8};

//Publishers
std_msgs::Bool suction_msg;
ros::Publisher suction_pub("/garbage_quick_sort/arduino/suction_active", &suction_msg); 

void globalStateCallback(const garbage_quick_sort_arduino::RobotState& msg)
{
  current_global_state = msg;
}

//Callback
ros::Subscriber<garbage_quick_sort_arduino::RobotState> global_state_sub("/garbage_quick_sort/global_state", &globalStateCallback);
ros::NodeHandle nh;

bool force_reading()
{
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
    
void setup() 
{
  pinMode(suction_pump, OUTPUT);
  Serial.begin(57600); // // Serial Communication is starting with 9600 of baudrate speed

  nh.initNode();
  nh.advertise(target_contact_pub);
  nh.subscribe(global_state_sub);
}

void loop() 
{
  nh.spinOnce();

  // if moving down and force sensor detects, latch on the suction
  if ((current_global_state.robot_state == suction_active_states[0]) && (!suction_active))
  {
    suction_active = force_reading();
  }
  
  // if in any of the other active states and suction was latched high, keep it latched high
  else if ((current_global_state.robot_state == suction_active_states[0]) || (current_global_state.robot_state == suction_active_states[1]) ||
            (current_global_state.robot_state == suction_active_states[2]) || (current_global_state.robot_state == suction_active_states[3]) ||
            (current_global_state.robot_state == suction_active_states[4]))
  {
    if(suction_active)
    {
      digitalWrite(suction_pump, HIGH);
    }

    else
    {
      digitalWrite(suction_pump, LOW);
      //reset suction_active
      if (suction_active) suction_active = false;
    }
  }
  
  // if not in active state, then suction LOW
  else
  {
    digitalWrite(suction_pump, LOW);
    //reset suction_active
    if (suction_active) suction_active = false;
  }
  
  // keep publishing suction message for state machine to change state!
  suction_msg.data = suction_active;
  suction_pub.publish(&suction_msg);  

  delay(1);
}
