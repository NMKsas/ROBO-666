#include "is_assembly_started.h"

namespace Fusion {

IsAssemblyStarted::IsAssemblyStarted(const std::string &name, 
const BT::NodeConfig &config) : SyncActionNode(name, config)
{
    // initialize status false 
    setOutput("is_started_out", false); 
}

BT::PortsList IsAssemblyStarted::providedPorts() {
    return { BT::InputPort<bool>("is_started_in"),
             BT::OutputPort<bool>("is_started_out")};
}

BT::NodeStatus IsAssemblyStarted::tick() {

    bool isStarted = getInput<int>("is_started_in").value(); 

    if (isStarted){
        ROS_INFO("Assembly process already started"); 
        return BT::NodeStatus::SUCCESS; 
    }
    ROS_INFO("Assembly process not started"); 
    return BT::NodeStatus::FAILURE; 

}

} // namespace Fusion 
