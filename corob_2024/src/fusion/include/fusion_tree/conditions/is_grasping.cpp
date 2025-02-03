#include "is_grasping.h"


namespace Fusion {

IsGrasping::IsGrasping(const std::string &name, 
const BT::NodeConfig &config) : SyncActionNode(name, config)
{   
    // initialize grasping status false 
    setOutput("grasp_status_out", false); 
}

BT::PortsList IsGrasping::providedPorts() {
    return { BT::InputPort<bool>("grasp_status_in"), 
             BT::OutputPort<bool>("grasp_status_out")};
}

BT::NodeStatus IsGrasping::tick() {

    auto status = getInput<bool>("grasp_status_in");

    if (status.value() == true){
        ROS_INFO("Grasping object");
        return BT::NodeStatus::SUCCESS; 
    }
    ROS_INFO("Not grasping"); 
    return BT::NodeStatus::FAILURE; 
}

} // namespace Fusion 
