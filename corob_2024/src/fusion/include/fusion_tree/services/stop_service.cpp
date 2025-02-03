#include "stop_service.h"

namespace Fusion {
    
StopService::StopService(ros::NodeHandle &handle, const std::string &node_name, 
           const BT::NodeConfig &conf): 
           BT::RosServiceNode<fusion::ControllerStop>(handle, node_name, conf)
{}

BT::PortsList StopService::providedPorts() {
    return  {};
}

void StopService::sendRequest(RequestType &request) { 
    ROS_INFO("Sending stop request!"); 
}

BT::NodeStatus StopService::onResponse(const ResponseType &rep) {

    ROS_INFO("Response received");
    if (rep.success){
      return BT::NodeStatus::SUCCESS; 
    } else 
      ROS_WARN("StopService didn't succeed!"); 
      return BT::NodeStatus::FAILURE; 
    }

} // namespace Fusion
