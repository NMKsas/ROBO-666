#include "is_task_done.h"

namespace Fusion {

IsTaskDone::IsTaskDone(const std::string &name, 
const BT::NodeConfig &config) : SyncActionNode(name, config)
{
}

BT::PortsList IsTaskDone::providedPorts() {
    return { BT::InputPort<int>("task_progress"),
             BT::InputPort<int>("task_id")};
}

BT::NodeStatus IsTaskDone::tick() {

    int16_t taskMask = getInput<int>("task_progress").value(); 
    int taskId = getInput<int>("task_id").value();

    // check if the corresponding digit is true 
    bool isDone = taskMask & (1 << taskId);

    if (isDone){
        ROS_INFO("Task %i already done",taskId); 
        return BT::NodeStatus::SUCCESS; 
    }
    ROS_INFO("Task %i not done",taskId); 
    return BT::NodeStatus::FAILURE; 


}

} // namespace Fusion 
