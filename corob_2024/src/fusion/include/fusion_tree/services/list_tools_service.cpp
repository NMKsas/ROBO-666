#include "list_tools_service.h"

namespace Fusion {
    
ListToolsService::ListToolsService(ros::NodeHandle &handle, const std::string &node_name, 
const BT::NodeConfig &conf) : 
BT::RosServiceNode<fusion::ListTools>(handle, node_name, conf)
{}  

BT::PortsList ListToolsService::providedPorts()
{
    return  {
        BT::InputPort<uint16_t>("tools")};
}

void ListToolsService::sendRequest(RequestType &request)
{
    getInput("tools", request.tool_mask); 
    ROS_INFO("List tools client: Sending request to list needed tools"); 
}

BT::NodeStatus ListToolsService::onResponse(const ResponseType &rep)
{
    ROS_INFO("Targets listed");
    return BT::NodeStatus::SUCCESS;
}

}