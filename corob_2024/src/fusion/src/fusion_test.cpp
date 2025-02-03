#include <ros/ros.h>
#include "std_msgs/String.h"
#include "fusion_tree/test_class.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  
  // BT::BehaviorTreeFactory factory; 
  TestClass testNode; 
  ros::waitForShutdown(); 

};

