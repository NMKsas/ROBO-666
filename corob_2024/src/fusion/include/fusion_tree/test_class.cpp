#include "test_class.h"


TestClass::TestClass() : command_nh_{}, priority_nh_{}, spinner_(2), stop_triggered(false) {
  
  std::cout << "Constructor start" << std::endl; 
  this->spinner_.start();  

  // Preferably only one nodehandle is needed ? 
  this->cmd_sub_ = command_nh_.subscribe<std_msgs::String>("/verified_command", 10, &TestClass::regularCallback, this);
  this->stop_sub_ = priority_nh_.subscribe<std_msgs::Bool>("/stop_enabled", 10, &TestClass::priorityCallback, this); 
  this->active_command = "idle";

  std::cout << "Constructor end" << std::endl; 
}

TestClass::~TestClass()
{
    this->spinner_.stop(); 
}

void TestClass::tick(){
  /**
  * Ideally here the Behavioral tree does the sequential tick() traverse 
  * over the tree. By nature, tick() is a blocking function 
  */

  status = "idle"; 

  while( ros::ok() && (status == "idle"|| status == "running"))
  {
    // Sequential stuff happening here 
    for (int i = 0; i < 10; ++i){
        // stop is received, break the loop 
        if (this->stop_triggered.load()){
            break; 
        }
        ros::Rate loop_rate(1);
        std::cout << "Sleeping...zzZzz..." << std::endl;  
        loop_rate.sleep(); 
    }
    break;
  }

  // for now, return the state back to idle, disable the stop   
  active_command = "idle"; 
  this->stop_triggered.store(false);
}



void TestClass::regularCallback(const std_msgs::String::ConstPtr &msg) {
  /**
   * This is a regular callback for commands, which start ticking the tree 
   */
  
  std::cout << "This is the received command: " << msg->data.c_str() << std::endl; 
  received_command = msg->data.c_str();
  
  if (active_command == "idle") {
    std::cout << "[ Robot is idle and stop is cleared, proceed to tick ]" << std::endl; 
    this->active_command = received_command; 
    return this->tick(); 
  } 
  std::cout << "[ Command not known, or other command is still processing ]" << std::endl;
  return;
}


void TestClass::priorityCallback(const std_msgs::Bool::ConstPtr &msg)
{
  /**
   * This is a priority callback, that HAS TO COME THROUGH even when the 
   * sequential tick() function is running 
   */
  this->stop_triggered.store(msg->data);
  std::cout << "PRIORITY: " << this->stop_triggered << std::endl; 
}
