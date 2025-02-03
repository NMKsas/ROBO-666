#ifndef FEEDBACK_ACTION_NODE_H
#define FEEDBACK_ACTION_NODE_H
#include "behaviortree_cpp/action_node.h"
#include <stdexcept>


namespace Fusion 
{
/**
 * Simple node class for giving feedback to the user, currently prints 
 * a message on console.  
 */
class GiveFeedback: public BT::SyncActionNode {
    public:
        GiveFeedback(const std::string& name, const BT::NodeConfig& config);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override; 
};


} // namespace Fusion 

#endif // FEEDBACK_ACTION_NODE_H