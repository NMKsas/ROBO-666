
#ifndef IS_LOCKED_H
#define IS_LOCKED_H
#include "behaviortree_cpp/action_node.h"
#include <stdexcept>
#include <ros/ros.h>

namespace Fusion {

class IsLocked : public BT::SyncActionNode {

    public:
        IsLocked(const std::string& name, 
                   const BT::NodeConfig& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
};

} // namespace Fusion 

#endif  // IS_LOCKED_H