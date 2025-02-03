#include "list_targets_service.h"

namespace Fusion {
    
ListTargetsService::ListTargetsService(ros::NodeHandle &handle, const std::string &node_name, 
const BT::NodeConfig &conf, std::string &action_name) : 
BT::RosServiceNode<fusion::ListTargets>(handle, node_name, conf), 
action_name_(action_name) 
{}  

BT::PortsList ListTargetsService::providedPorts()
{
    return  {
        BT::InputPort<std::string>("target_type")};
}

void ListTargetsService::sendRequest(RequestType &request)
{
    getInput("target_type", request.target_type); 
    request.action = action_name_; 
    ROS_INFO("Sending request to list %ss", request.target_type.c_str()); 
    ROS_INFO("%s", request.action.c_str());
}

BT::NodeStatus ListTargetsService::onResponse(const ResponseType &rep)
{
    ROS_INFO("Targets listed");
    return BT::NodeStatus::SUCCESS;
}

}