#include "is_fault_service.h"

namespace Fusion {
    
IsFaultService::IsFaultService(ros::NodeHandle &handle, 
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::IsFault>(handle, node_name, conf)
{}


BT::PortsList IsFaultService::providedPorts() {
return  {
        BT::OutputPort<std::string>("feedback"),
        BT::OutputPort<std::string>("recovery")
        };
}

void IsFaultService::sendRequest(RequestType &request) {
    
    ROS_INFO("Fault service client: Sending request for checking faults"); 
}

BT::NodeStatus IsFaultService::onResponse(const ResponseType &rep) {

    ROS_INFO("Fault service client: response received");

    if (rep.is_fault == true){
        ROS_ERROR("Fault service client: a fault has been detected!"); 
        setOutput<std::string>("feedback", rep.fault);
        setOutput<std::string>("recovery", rep.recovery); 
        return BT::NodeStatus::SUCCESS; 
    }
    ROS_INFO("Fault service client: no faults detected"); 
    return BT::NodeStatus::FAILURE; 
    
}

}