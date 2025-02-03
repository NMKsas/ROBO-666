#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"
#include "fusion_tree/fusion_tree.h"
#include "std_msgs/String.h"


using namespace BT;
using namespace Fusion; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_behavior_tree");
  
  BT::BehaviorTreeFactory factory; 
  Fusion::FusionTree fusion_tree_node(factory, DEFAULT_INIT_CONFIG); 
  ros::waitForShutdown(); 

};

