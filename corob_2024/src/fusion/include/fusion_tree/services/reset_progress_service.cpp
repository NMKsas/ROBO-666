#include "reset_progress_service.h"

namespace Fusion {
    
ResetProgressService::ResetProgressService(ros::NodeHandle& handle, 
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::ResetProgress>(handle, node_name, conf)
{
}

BT::PortsList ResetProgressService::providedPorts()
{
    return  {};
}

void ResetProgressService::sendRequest(RequestType &request)
{ 
    ROS_INFO("Sending request to reset task progress"); 
}

BT::NodeStatus ResetProgressService::onResponse(const ResponseType &rep)
{
    ROS_INFO("Reset progress client: response received");
    bool success = rep.success; 

    if (success == true){
      return BT::NodeStatus::SUCCESS; 
    }
    ROS_WARN("Reset progress client: reset failed!");
    return BT::NodeStatus::FAILURE; 

}

}