
#ifndef IS_COMMAND_H
#define IS_COMMAND_H
#include "behaviortree_cpp/action_node.h"
#include <stdexcept>
#include <ros/ros.h>

namespace Fusion {

class IsCommand : public BT::SyncActionNode {

    public:
        IsCommand(const std::string& name, 
                  const BT::NodeConfig& config,
                  std::string& command);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
    private: 
        std::string* command; 
};

} // namespace Fusion 

#endif  // IS_COMMAND_H