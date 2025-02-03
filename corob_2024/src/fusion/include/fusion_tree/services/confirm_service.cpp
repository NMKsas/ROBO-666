#include "confirm_service.h"

namespace Fusion {
    
ConfirmService::ConfirmService(ros::NodeHandle &handle, 
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::Confirm>(handle, node_name, conf)
{}

BT::PortsList ConfirmService::providedPorts()
{
    return  {
        BT::InputPort<int>("num_attempts"), 
        BT::OutputPort<bool>("is_confirmed")};
}

void ConfirmService::sendRequest(RequestType &request)
{ 
    if( !getInput<int>("num_attempts", this->num_attempts_)) {
      ROS_WARN("[Confirm target client] Ports are not defined correctly"); 
    }
    // set values from inputs
    request.num_attempts = this->num_attempts_; 

    ROS_INFO("Sending request for to confirm"); 
    
}

BT::NodeStatus ConfirmService::onResponse(const ResponseType &rep)
{
    ROS_INFO("Confirm service: response received");

    setOutput<bool>("is_confirmed", rep.isConfirmed);
    if (rep.isConfirmed == true){
      ROS_INFO("Confirmation received"); 
      return BT::NodeStatus::SUCCESS; 
    } else {
      ROS_INFO("No confirmation received"); 
      return BT::NodeStatus::FAILURE; 
    }
}

}