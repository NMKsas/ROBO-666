#include "task_progress_service.h"

namespace Fusion {
    
TaskProgressService::TaskProgressService(ros::NodeHandle &handle, 
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::TaskProgress>(handle, node_name, conf)
{
    // initialize task progress zero 
    setOutput("task_progress_output", 0); 
}

BT::PortsList TaskProgressService::providedPorts()
{
    return  {
        BT::OutputPort<uint16_t>("task_progress_output")};
}

void TaskProgressService::sendRequest(RequestType &request)
{ 
    ROS_INFO("Sending request for task progress");   
}

BT::NodeStatus TaskProgressService::onResponse(const ResponseType &rep)
{
    ROS_INFO("Task progress service: response received");
    setOutput<uint16_t>("task_progress_output", rep.task_progress);
    return BT::NodeStatus::SUCCESS; 
}

}