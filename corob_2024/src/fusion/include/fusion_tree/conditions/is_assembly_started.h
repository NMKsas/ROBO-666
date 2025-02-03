
#ifndef IS_ASSEMBLY_STARTED
#define IS_ASSEMBLY_STARTED
#include "behaviortree_cpp/action_node.h"
#include <stdexcept>
#include <ros/ros.h>

namespace Fusion {

class IsAssemblyStarted : public BT::SyncActionNode {

    public:
        IsAssemblyStarted(const std::string& name, 
                   const BT::NodeConfig& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
};

} // namespace Fusion 

#endif  // IS_ASSEMBLY_STARTED