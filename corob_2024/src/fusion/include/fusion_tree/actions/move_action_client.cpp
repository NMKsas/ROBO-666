#include "move_action_client.h"

namespace Fusion {

MoveActionClient::MoveActionClient(ros::NodeHandle &handle, 
           const std::string &name, const BT::NodeConfig &conf) :
           BT::RosActionNode<fusion::MoveAction>(handle, name, conf)
{
}

BT::PortsList MoveActionClient::providedPorts() {
    return  {BT::InputPort<int16_t>("direction"), 
             BT::OutputPort<std::string>("status_msg"),
};
}

bool MoveActionClient::sendGoal(GoalType &goal)
{      
    /**
     * Send goal to Move action server.
     */
    auto direction = getInput<int16_t>("direction"); 

    // the value for grasping status should exist; this check is probably 
    // unnecessary
    if (!direction) {
        throw std::runtime_error("Move action client: error reading port [direction]"); 
    }
    goal.target_id = direction.value();  
    std::cout << goal.target_id << std::endl; 
    ROS_INFO("Move action client: sending request to move, direction ID %d", direction.value());
    return true; 
}

BT::NodeStatus MoveActionClient::onResult(const ResultType &res)
{
    /**
     * Result from the action server is received. Set output status. 
     */

    ROS_INFO("Move action client: result received");
    if (res.success == true){
        setOutput("status_msg",  this->feedback_->feedback_msg.c_str()); 
        return BT::NodeStatus::SUCCESS; 
    }
    setOutput("status_msg", "Move action resulted in failure."); 
    return BT::NodeStatus::FAILURE; 
}

BT::NodeStatus MoveActionClient::onFailedRequest(FailureCause failure) {
    /**
     * Action fails; report the cause in status message 
    */

    ROS_ERROR("Move action client: Move request failed %d", static_cast<int>(failure));
    std::string cause; 
    if (failure == MISSING_SERVER){
        cause = "The control server is missing."; 
    } else if (failure == ABORTED_BY_SERVER){
        cause = "The goal was aborted by the control server."; 
    } else if (failure == REJECTED_BY_SERVER){
        cause = "The goal was rejected by the control server."; 
    }
    setOutput("status_msg", "Move action request failed. " + cause); 
    return BT::NodeStatus::FAILURE;

}
BT::NodeStatus MoveActionClient::onRunning() {
    /**
     * The node is running; update the grasping status as the action proceeds  
     */
    // Feedback has been updated 
    if (this->feedback_){
        ROS_INFO("Move action client: %s", this->feedback_->feedback_msg.c_str()); 
    }
    return BT::NodeStatus::RUNNING;
}

void MoveActionClient::halt() {
    /**
     * Halt the running action 
     */   

    if( status() == BT::NodeStatus::RUNNING ) {
      ROS_WARN("Move action client: action halted");
      setOutput("status_msg","Move action halted."); 
      BaseClass::halt();
    }
}

} // namespace Fusion