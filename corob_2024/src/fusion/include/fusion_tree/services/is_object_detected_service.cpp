#include "is_object_detected_service.h"

namespace Fusion {
    
IsObjectDetectedService::IsObjectDetectedService(ros::NodeHandle &handle, 
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::IsObjectDetected>(handle, node_name, conf)
{}


BT::PortsList IsObjectDetectedService::providedPorts()
{
    return  {};
}

void IsObjectDetectedService::sendRequest(RequestType &request)
{
    ROS_INFO("Sending request for detected objects"); 
}

BT::NodeStatus IsObjectDetectedService::onResponse(const ResponseType &rep)
{
    ROS_INFO("Detected objects response received");

    if (rep.is_detected == true){
      ROS_INFO("Objects detected"); 
      return BT::NodeStatus::SUCCESS; 
    } else {
      ROS_ERROR("No objects detected"); 
      return BT::NodeStatus::FAILURE; 
    }
}



}