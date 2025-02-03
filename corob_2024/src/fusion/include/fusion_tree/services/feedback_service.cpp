#include "feedback_service.h"

namespace Fusion {

FeedbackService::FeedbackService(ros::NodeHandle& handle,
const std::string &node_name, const BT::NodeConfig &conf): 
BT::RosServiceNode<fusion::Feedback>(handle, node_name, conf)
{
} 

BT::PortsList FeedbackService::providedPorts() {
    return  {
        BT::InputPort<bool>("repeat_feedback"),
        BT::InputPort<std::string>("message"),
        };
}

void FeedbackService::sendRequest(RequestType &request) {
    
    auto message_input = getInput<std::string>("message"); 
    auto repeat_feedback_input = getInput<bool>("repeat_feedback"); 

    std::string message = ""; 
    bool repeat_previous = false; 
    if (message_input) {
        ROS_INFO("Feedback service client: An empty feedback message given"); 
        message = message_input.value(); 
    }
    if (repeat_feedback_input) {
        ROS_INFO("Feedback service client: Repeat flag set"); 
        repeat_previous = repeat_feedback_input.value();  
    }
    request.repeat_previous = repeat_previous;
    request.message = message; 

    ROS_INFO("%s", message.c_str()); 

}

BT::NodeStatus FeedbackService::onResponse(const ResponseType &rep) {
    ROS_INFO("Feedback given"); 
    return BT::NodeStatus::SUCCESS; 
}

}