#ifndef FUSION_TREE_H
#define FUSION_TREE_H

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "feedback_action_node.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include <iostream> 
#include <boost/filesystem.hpp> 
#include <ros/ros.h>
#include <vector>
#include <unordered_set>
#include <unordered_map>

#include "behaviortree_ros/bt_custom_register.h"
#include "behaviortree_ros/bt_service_node.h"

#include "fusion_tree/services/confirm_service.h"
#include "fusion_tree/services/confirm_target_service.h"
#include "fusion_tree/services/feedback_service.h"
#include "fusion_tree/services/get_task_tools_service.h"
#include "fusion_tree/services/is_fault_service.h"
#include "fusion_tree/services/is_object_detected_service.h"
#include "fusion_tree/services/list_targets_service.h"
#include "fusion_tree/services/list_tools_service.h"
#include "fusion_tree/services/reset_progress_service.h"
#include "fusion_tree/services/snap_to_tool_service.h"
#include "fusion_tree/services/stop_service.h"
#include "fusion_tree/services/task_inspection_service.h"
#include "fusion_tree/services/task_instructions_service.h"
#include "fusion_tree/services/task_progress_service.h"
#include "fusion_tree/services/verify_task_service.h"
#include "fusion_tree/services/gripper_service.h"

#include "fusion_tree/actions/give_action_client.h"
#include "fusion_tree/actions/move_action_client.h"
#include "fusion_tree/actions/pick_action_client.h"
#include "fusion_tree/actions/place_action_client.h"
#include "fusion_tree/actions/snap_action_client.h"

#include "fusion_tree/conditions/is_assembly_started.h"
#include "fusion_tree/conditions/is_command.h"
#include "fusion_tree/conditions/is_grasping.h"
#include "fusion_tree/conditions/is_locked.h"
#include "fusion_tree/conditions/is_task_done.h"

static const Fusion::locked_target DEFAULT_LOCATION = {-1, "default", geometry_msgs::PoseWithCovariance()};
static const Fusion::locked_target DEFAULT_OBJECT =  {-1, "default", geometry_msgs::PoseWithCovariance()};

struct state_booleans {
    bool is_grasping;         
    bool is_object_locked;
    bool is_stop_disabled; 
    bool is_location_locked; 
}; 

struct init_config { 
    std::string active_command; 
    std::string tree_name;
    state_booleans state_flags;
    std::unordered_set<std::string> detected_objects;      
}; 

static state_booleans const DEFAULT_STATE_FLAGS = {false, false, true, false}; 
static init_config const DEFAULT_INIT_CONFIG = { "idle", "MainTreeAssembly", DEFAULT_STATE_FLAGS}; 

namespace Fusion 
{

class FusionTree{
    public: 
        FusionTree(BT::BehaviorTreeFactory& factory, init_config config); 
        ~FusionTree(); 

        // state control 
        BT::NodeStatus HaltTree();
        BT::NodeStatus IsStopCommand();
        BT::NodeStatus Idle();

        void Tick();  
        void callback(const std_msgs::String::ConstPtr& msg); 
        void stopCallback(const std_msgs::Bool::ConstPtr& msg); 

    private: 
        
        ros::AsyncSpinner spinner_; 
        ros::NodeHandle priority_nh_; 
        ros::NodeHandle command_nh_;
        ros::Subscriber cmd_sub_; 
        ros::Subscriber stop_sub_;
        BT::Tree fusion_tree_; 
        
        std::string active_command_;
        std::atomic<bool> is_stop_disabled_;

        Fusion::locked_target locked_location_; 
        Fusion::locked_target locked_object_; 

};


}  // namespace Fusion

#endif // FUSION_TREE_H