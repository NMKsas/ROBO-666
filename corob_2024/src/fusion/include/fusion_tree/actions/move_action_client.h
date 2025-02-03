#ifndef MOVE_ACTION_CLIENT_H
#define MOVE_ACTION_CLIENT_H

#include "behaviortree_ros/bt_action_node.h"
#include "behaviortree_ros/bt_custom_register.h"
#include <ros/ros.h>
#include "fusion/MoveAction.h"
#include <tuple> 
#include <unordered_map>

namespace Fusion {

class MoveActionClient: public BT::RosActionNode<fusion::MoveAction> {

    public:
        MoveActionClient( ros::NodeHandle& handle, const std::string& name, 
                            const BT::NodeConfig& conf);   

        static BT::PortsList providedPorts();

        bool sendGoal(GoalType& goal) override;

        BT::NodeStatus onResult( const ResultType& res) override;

        virtual BT::NodeStatus onFailedRequest(FailureCause failure) override;

        virtual BT::NodeStatus onRunning() override; 
    
        void halt() override;
    
};

} // namespace fusion

#endif // MOVE_ACTION_CLIENT_H