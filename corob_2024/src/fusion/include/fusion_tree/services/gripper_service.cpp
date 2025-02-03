#include "gripper_service.h"

namespace Fusion {
    
GripperService::GripperService(ros::NodeHandle &handle, 
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::ActivateGripper>(handle, node_name, conf)
{}

BT::PortsList GripperService::providedPorts()
{
    return  {
        BT::InputPort<bool>("is_opening"), // true - open signal; false - close
        BT::InputPort<bool>("grasp_status_in"), 
        BT::OutputPort<bool>("grasp_status_out")};
}

void GripperService::sendRequest(RequestType &request)
{

    if( !getInput<bool>("is_opening", this->is_open_)) {
        ROS_WARN("[Gripper service client] Ports are not defined correctly"); 
    }
    request.activate = true; 
    ROS_INFO("Sending request for to confirm"); 
}

BT::NodeStatus GripperService::onResponse(const ResponseType &rep)
{
    ROS_INFO("Gripper service: response received");

    if (rep.success == true){
        ROS_INFO("Gripper service success received");
        setOutput<bool>("grasp_status_out", !this->is_open_);
        return BT::NodeStatus::SUCCESS; 
    } else {
        // this should never happen; if it does
        // what are the counter-measures? -> faulty state and its clearance?  
        ROS_WARN("Gripper service failure!"); 
        return BT::NodeStatus::FAILURE; 
    }
}

}