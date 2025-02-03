#include "snap_to_tool_service.h"

namespace Fusion {
    
SnapToToolService::SnapToToolService(ros::NodeHandle &handle, 
const std::string &node_name, const BT::NodeConfig &conf, 
locked_target &locked_target) : 
BT::RosServiceNode<fusion::SnapToTool>(handle, node_name, conf),
locked_target_(&locked_target)
{}


BT::PortsList SnapToToolService::providedPorts()
{
    return  {
        BT::InputPort<uint8_t>("tool_id"),
        BT::OutputPort<std::string>("status_msg") 
    };
}

void SnapToToolService::sendRequest(RequestType &request)
{
    getInput("tool_id", request.tool_id); 
    this->locked_target_->target_id = request.tool_id;
    ROS_INFO("Snap to tool service: sending request for tool"); 
}

BT::NodeStatus SnapToToolService::onResponse(const ResponseType &rep)
{

    ROS_INFO("Snap to tool service: result received"); 
    if (rep.success){
        setOutput<std::string>("status_msg", "Giving the " + rep.tool_name);
        this->locked_target_->pose_msg = rep.target_pose; 
        return BT::NodeStatus::SUCCESS; 
    }
    setOutput<std::string>("status_msg", "The " + rep.tool_name + " not detected. Cannot pick it.");
    return BT::NodeStatus::FAILURE; 

}


}