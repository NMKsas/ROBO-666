#include "snap_action_client.h"

namespace Fusion {

SnapToTargetClient::SnapToTargetClient(ros::NodeHandle &handle, 
           const std::string &name, const BT::NodeConfig &conf,
           locked_target &locked_target) :
     BT::RosActionNode<fusion::SnapAction>(handle, name, conf),
     locked_target_(&locked_target)           
{
}

BT::PortsList SnapToTargetClient::providedPorts()
{
    return  {
        BT::InputPort<std::string>("strategy"),
        BT::InputPort<std::string>("target_type"),
        BT::OutputPort<int16_t>("target_id_out")
    };
}

bool SnapToTargetClient::sendGoal(GoalType &goal)
{
    if ((!getInput<std::string>("target_type", goal.target_type)) &&
        (!getInput<std::string>("strategy", goal.strategy))){
        return false; 
    }
    ROS_INFO("Snap to target action: sending request"); 
    return true; 
}

BT::NodeStatus SnapToTargetClient::onResult(const ResultType &res)
{
    ROS_INFO("Snap to target action: result received"); 
    locked_target_->target_id = res.target_id;
    locked_target_->pose_msg = res.target_pose;
    setOutput("target_id_out", res.target_id); 
    return BT::NodeStatus::SUCCESS; 
}

BT::NodeStatus SnapToTargetClient::onFailedRequest(FailureCause failure)
{
    /**
     * Action fails; report the cause in status message 
     */

    ROS_ERROR("Snap to target request failed %d", static_cast<int>(failure));
    std::string cause; 
    if (failure == MISSING_SERVER){
        cause = "The control server is missing."; 
    } else if (failure == ABORTED_BY_SERVER){
        cause = "The goal was aborted by the snap server."; 
    } else if (failure == REJECTED_BY_SERVER){
        cause = "The goal was rejected by the snap server."; 
    }
    setOutput("status_msg", "Snap request failed. " + cause); 
    return BT::NodeStatus::FAILURE;
}

void SnapToTargetClient::halt_(){
    
    if( status() == BT::NodeStatus::RUNNING ) {
      ROS_WARN("Snap to target action halted");
      BaseClass::halt();
    }
}

} // namespace Fusion 