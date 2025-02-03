#include "place_action_client.h"

namespace Fusion {

PlaceActionClient::PlaceActionClient(ros::NodeHandle &handle, 
           const std::string &name, const BT::NodeConfig &conf,
           Fusion::locked_target &locked_target) :
     BT::RosActionNode<fusion::PlaceAction>(handle, name, conf),
     locked_target_(&locked_target)           
{
}

BT::PortsList PlaceActionClient::providedPorts() {
    return  {BT::InputPort<bool>("grasp_status_in"), 
             BT::OutputPort<bool>("grasp_status_out"),
             BT::OutputPort<std::string>("status_msg")};
}

bool PlaceActionClient::sendGoal(GoalType &goal) {
    /**
     * Send goal to Place action server.
     */       
    
    auto status = getInput<bool>("grasp_status_in"); 

    if (!status) {
        throw std::runtime_error("error reading port [message]"); 
    } else {
        this->is_grasping_ = status.value(); 
    }

    geometry_msgs::PoseWithCovariance& pose_msg = locked_target_->pose_msg;
    goal.pose = pose_msg.pose;  

    ROS_INFO("Place action call: sending request");
    return true; 

}

BT::NodeStatus PlaceActionClient::onResult(const ResultType &res) {
    /**
     * Result from the action server is received. Set grasping status and 
     * output action status. 
     */
    ROS_INFO("Place action: result received"); 
    if (res.success == true){
        setOutput("grasp_status_out", false); 
        setOutput("status_msg", "Place action succeeded.");
        return BT::NodeStatus::SUCCESS; 
    }
    setOutput("status_msg", "Place action resulted in failure."); 
    return BT::NodeStatus::FAILURE; 
}

BT::NodeStatus PlaceActionClient::onFailedRequest(FailureCause failure) {
    /**
     * Action fails; report the cause in status message 
     */

    ROS_ERROR("Place request failed %d", static_cast<int>(failure));
    std::string cause; 
    if (failure == MISSING_SERVER){
        cause = "The control server is missing."; 
    } else if (failure == ABORTED_BY_SERVER){
        cause = "The goal was aborted by the control server."; 
    } else if (failure == REJECTED_BY_SERVER){
        cause = "The goal was rejected by the control server."; 
    }
    setOutput("status_msg", "Place request failed. " + cause); 
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PlaceActionClient::onRunning() {
    /**
     * The node is running; update the grasping status as the action proceeds  
     */

    // Feedback has been updated 
    if (this->feedback_){
        // get grasping status 
        this->is_grasping_ = this->feedback_->is_grasping; 
        ROS_INFO("Place action feedback: %s", this->feedback_->feedback_msg.c_str()); 
    }
    // set grasping status to blackboard 
    setOutput("grasp_status_out", this->is_grasping_);
    return BT::NodeStatus::RUNNING;
}

void PlaceActionClient::halt(){
    /**
     * Halt the running action 
     */   
    
    if( status() == BT::NodeStatus::RUNNING ) {
      ROS_WARN("Place action halted");
      setOutput("status_msg", "Place action halted."); 
      BaseClass::halt();
    }
}

} // namespace Fusion