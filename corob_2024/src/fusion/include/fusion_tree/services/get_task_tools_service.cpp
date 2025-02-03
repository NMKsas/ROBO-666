#include "get_task_tools_service.h"

namespace Fusion {
    
GetTaskToolsService::GetTaskToolsService(ros::NodeHandle &handle, 
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::GetTools>(handle, node_name, conf)
{}


BT::PortsList GetTaskToolsService::providedPorts() {
return  {
        BT::OutputPort<uint16_t>("tools"),
        BT::OutputPort<BT::SharedQueue<uint8_t>>("tool_queue")
        };
}

void GetTaskToolsService::sendRequest(RequestType &request) {
    
    ROS_INFO("Task tools client: Sending request for currently needed tools"); 
}

BT::NodeStatus GetTaskToolsService::onResponse(const ResponseType &rep) {

    ROS_INFO("Task tools client: response received");
    auto shared_queue = std::make_shared<std::deque<uint8_t>>(); 
    setOutput<uint16_t>("tools", rep.tool_mask);

    // no tools needed
    if (rep.tool_mask == 0){
        setOutput("tool_queue", shared_queue); 
        return BT::NodeStatus::FAILURE; 
    }

    for (uint8_t i = 0; i < 16; ++i) { 
        if (rep.tool_mask & (1 << i)) { 
            shared_queue->push_back(i); 
        }
    } 
    setOutput("tool_queue", shared_queue); 
    return BT::NodeStatus::SUCCESS; 
    
}
}