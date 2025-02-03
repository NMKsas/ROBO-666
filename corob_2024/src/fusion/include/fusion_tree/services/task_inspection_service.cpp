#include "task_inspection_service.h"

namespace Fusion {
    
TaskInspectionService::TaskInspectionService(ros::NodeHandle& handle, 
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::Inspection>(handle, node_name, conf)
{
    // initialize task progress zero 
    setOutput("task_success", false); 
}

BT::PortsList TaskInspectionService::providedPorts()
{
    return  {
        BT::OutputPort<bool>("task_success"), 
        BT::OutputPort<std::string>("feedback"),
        BT::OutputPort<std::string>("recovery")};
}

void TaskInspectionService::sendRequest(RequestType &request)
{ 
    ROS_INFO("Sending request to inspect the on-going task");
}

BT::NodeStatus TaskInspectionService::onResponse(const ResponseType &rep)
{
    ROS_INFO("Task progress service: response received");
    bool success = rep.task_success; 
    setOutput("recovery", rep.recovery);
    setOutput("feedback", rep.feedback);
    setOutput("task_success", success);
    if (success == true){
      return BT::NodeStatus::SUCCESS; 
    }
    return BT::NodeStatus::FAILURE; 
}

}