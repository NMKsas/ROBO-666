#include "pick_action_client.h"

// {id, (width, force, .z)}
const static std::unordered_map<int, std::tuple<double, double, double>> OBJECT_DICT = {
  {0 , std::make_tuple(0.005, 20.0, 0.119)}, // allen key 
  {2 , std::make_tuple(0.005, 20.0, 0.118)}  // big bolt
}; 

namespace Fusion {

PickActionClient::PickActionClient(ros::NodeHandle &handle, 
           const std::string &name, const BT::NodeConfig &conf,
           Fusion::locked_target &locked_target) :
     BT::RosActionNode<fusion::PickAction>(handle, name, conf),
     locked_target_(&locked_target)           
{
}

BT::PortsList PickActionClient::providedPorts() {
    return  {BT::InputPort<bool>("grasp_status_in"), 
             BT::OutputPort<bool>("grasp_status_out"), 
             BT::OutputPort<std::string>("status_msg"),
};
}

bool PickActionClient::sendGoal(GoalType &goal)
{      
    /**
     * Send goal to Pick action server.
     */
    auto grasp_status_in = getInput<bool>("grasp_status_in"); 

    // the value for grasping status should exist; this check is probably 
    // unnecessary
    if (!grasp_status_in) {
        throw std::runtime_error("error reading port [message]"); 
    } else {
        this->is_grasping_ = grasp_status_in.value(); 
    }

    auto& id = this->locked_target_->target_id; 
    goal.pose = locked_target_->pose_msg.pose;
    
    ROS_INFO("Pick action call: sending request");
    return true; 

}

BT::NodeStatus PickActionClient::onResult(const ResultType &res)
{
    /**
     * Result from the action server is received. Set grasping status and 
     * output action status. 
     */

    ROS_INFO("Pick action: result received"); 
    if (res.success == true){
        setOutput("grasp_status_out", true);
        setOutput("status_msg", "Pick action succeeded."); 
        return BT::NodeStatus::SUCCESS; 
    }
    setOutput("status_msg", "Pick action resulted in failure."); 
    return BT::NodeStatus::FAILURE; 
}

BT::NodeStatus PickActionClient::onFailedRequest(FailureCause failure) {
    /**
     * Action fails; report the cause in status message 
    */

    ROS_ERROR("Pick request failed %d", static_cast<int>(failure));
    std::string cause; 
    if (failure == MISSING_SERVER){
        cause = "The control server is missing."; 
    } else if (failure == ABORTED_BY_SERVER){
        cause = "The goal was aborted by the control server."; 
    } else if (failure == REJECTED_BY_SERVER){
        cause = "The goal was rejected by the control server."; 
    }
    setOutput("status_msg", "Pick request failed. " + cause); 
    return BT::NodeStatus::FAILURE;

}
BT::NodeStatus PickActionClient::onRunning() {
    /**
     * The node is running; update the grasping status as the action proceeds  
     */

    // Feedback has been updated 
    if (this->feedback_){
        // get grasping status 
        this->is_grasping_ = this->feedback_->is_grasping; 
        ROS_INFO("Pick action feedback: %s", this->feedback_->feedback_msg.c_str()); 
    }
    // set grasping status to blackboard 
    setOutput("grasp_status_out", this->is_grasping_);
    return BT::NodeStatus::RUNNING;
}

void PickActionClient::halt() {
    /**
     * Halt the running action 
     */   

    if( status() == BT::NodeStatus::RUNNING ) {
      ROS_WARN("Pick action halted");
      setOutput("status_msg","Pick action halted."); 
      BaseClass::halt();
    }
}

} // namespace Fusion