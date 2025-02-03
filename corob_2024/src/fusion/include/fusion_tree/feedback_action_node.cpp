#include "feedback_action_node.h"


namespace Fusion 
{

GiveFeedback::GiveFeedback(const std::string& name, const BT::NodeConfig& config):
SyncActionNode(name,config)
{}

BT::PortsList GiveFeedback::providedPorts()
{
    const char* msg_description = "Message delivered as feedback";
    return { BT::InputPort<std::string>("message", msg_description)};
}

BT::NodeStatus GiveFeedback::tick()
{
    auto msg = getInput<std::string>("message"); 

    if (!msg) {
        throw std::runtime_error("error reading port [message]"); 
    }

    std::string  message =  msg.value(); 
    std::cout << "[ " << message << " ]" << std::endl; 

    return BT::NodeStatus::SUCCESS; 
}

}