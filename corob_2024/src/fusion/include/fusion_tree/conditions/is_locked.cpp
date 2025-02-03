#include "is_locked.h"


namespace Fusion {

IsLocked::IsLocked(const std::string &name, 
const BT::NodeConfig &config) : SyncActionNode(name, config)
{
    // initialize lock status false 
    setOutput("lock_status_out", false); 
}

BT::PortsList IsLocked::providedPorts() {
    return { BT::InputPort<bool>("lock_status_in"), 
             BT::OutputPort<bool>("lock_status_out")};
}

BT::NodeStatus IsLocked::tick() {

    auto status = getInput<bool>("lock_status_in");

    if (!status) {
        // No previous status exists in the blackboard, assume not locked 
        setOutput("lock_status_out", false); 
        ROS_INFO("Not locked"); 
        return BT::NodeStatus::FAILURE;   
    } 

    if (status.value() == true){
        ROS_INFO("Locked target");
        return BT::NodeStatus::SUCCESS; 
    }
    ROS_INFO("No locked target"); 
    return BT::NodeStatus::FAILURE; 
}

} // namespace Fusion 
