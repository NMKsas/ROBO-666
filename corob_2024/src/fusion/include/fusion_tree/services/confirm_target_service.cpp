#include "confirm_target_service.h"

namespace Fusion {

ConfirmTargetService::ConfirmTargetService(ros::NodeHandle &handle, 
const std::string &node_name, const BT::NodeConfig &conf) :
BT::RosServiceNode<fusion::ConfirmTarget>(handle, node_name, conf)
{}

BT::PortsList ConfirmTargetService::providedPorts()
{
    return  { 
        BT::InputPort<int16_t>("target_id_in"),
        BT::InputPort<std::string>("target_type"),
        BT::InputPort<int16_t>("num_attempts"),
        BT::OutputPort<bool>("is_confirmed") };
}

void ConfirmTargetService::sendRequest(RequestType &request)
{ 

    if (!getInput<int16_t>("target_id_in", request.target_id) ||
        !getInput<std::string>("target_type", request.target_type) ||
        !getInput<int16_t>("num_attempts", request.num_attempts)) {
        ROS_WARN("[Confirm target client]: Ports are not defined correctly"); 
    }

    ROS_INFO("[Confirm target client]: Sending request for confirming %s with ID %d", 
              request.target_type.c_str(), request.target_id);     
}

BT::NodeStatus ConfirmTargetService::onResponse(const ResponseType &rep)
{
    ROS_INFO("[Confirm target client]: Response received");

    setOutput<bool>("is_confirmed", rep.isConfirmed);
    if (rep.isConfirmed == true){
      ROS_INFO("Confirming target succeeded"); 
      return BT::NodeStatus::SUCCESS; 
    } else {
      ROS_ERROR("Confirming target failed."); 
      return BT::NodeStatus::FAILURE; 
    }
}

}