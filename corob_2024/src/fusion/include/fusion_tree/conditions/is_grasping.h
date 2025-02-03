
#ifndef IS_GRASPING_H
#define IS_GRASPING_H
#include "behaviortree_cpp/action_node.h"
#include <stdexcept>
#include <ros/ros.h>

namespace Fusion {

class IsGrasping : public BT::SyncActionNode {

    public:
        IsGrasping(const std::string& name, 
                   const BT::NodeConfig& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
};

} // namespace Fusion 

#endif  // IS_GRASPING_H