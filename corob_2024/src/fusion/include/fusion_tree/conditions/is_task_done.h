
#ifndef IS_TASK_DONE_H
#define IS_TASK_DONE_H
#include "behaviortree_cpp/action_node.h"
#include <stdexcept>
#include <ros/ros.h>

namespace Fusion {

class IsTaskDone : public BT::SyncActionNode {

    public:
        IsTaskDone(const std::string& name, 
                   const BT::NodeConfig& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
};

} // namespace Fusion 

#endif  // IS_TASK_DONE_H