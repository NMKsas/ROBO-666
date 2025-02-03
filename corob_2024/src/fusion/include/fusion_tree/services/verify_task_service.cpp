#include "verify_task_service.h"

namespace Fusion {
    
VerifyTaskService::VerifyTaskService(ros::NodeHandle& handle, 
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::VerifyTask>(handle, node_name, conf)
{
}

BT::PortsList VerifyTaskService::providedPorts()
{
    return  {};
}

void VerifyTaskService::sendRequest(RequestType &request)
{ 
    ROS_INFO("Verify task client: Sending request to verify the current task"); 
}

BT::NodeStatus VerifyTaskService::onResponse(const ResponseType &rep)
{
    ROS_INFO("Verify task client: Response received");
    bool success = rep.task_verified; 

    if (success == true){
      return BT::NodeStatus::SUCCESS; 
    }
    ROS_WARN("Verify task client: Verifying task failed!");
    return BT::NodeStatus::FAILURE; 

}

}