#include "task_instructions_service.h"

namespace Fusion {
    
TaskInstructionsService::TaskInstructionsService(ros::NodeHandle &handle, 
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::TaskInstructions>(handle, node_name, conf)
{
}

BT::PortsList TaskInstructionsService::providedPorts()
{
    return  {
        BT::InputPort<uint16_t>("task_progress_input"), 
    };
}

void TaskInstructionsService::sendRequest(RequestType &request)
{ 
    auto task_progress_input = getInput<uint16_t>("task_progress_input"); 

    if (!task_progress_input) {
        ROS_WARN("Task instructions service: Task ID not set!"); 
        return;
    }
    request.task_progress = task_progress_input.value(); 

    ROS_INFO("Task instructions service: sending request for instructions");
}

BT::NodeStatus TaskInstructionsService::onResponse(const ResponseType &rep)
{
    ROS_INFO("Task instructions service: response received");
    return BT::NodeStatus::SUCCESS; 
}

}