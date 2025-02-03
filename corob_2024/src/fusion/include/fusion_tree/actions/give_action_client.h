#ifndef GIVE_ACTION_CLIENT_H
#define GIVE_ACTION_CLIENT_H

#include "behaviortree_ros/bt_action_node.h"
#include "behaviortree_ros/bt_custom_register.h"
#include <ros/ros.h>
#include "fusion/GiveAction.h"
#include <tuple> 
#include <unordered_map>

namespace Fusion {

class GiveActionClient: public BT::RosActionNode<fusion::GiveAction> {

    public:
        GiveActionClient( ros::NodeHandle& handle, const std::string& name, 
                            const BT::NodeConfig& conf, 
                            Fusion::locked_target& locked_target);   

        static BT::PortsList providedPorts();

        bool sendGoal(GoalType& goal) override;

        BT::NodeStatus onResult( const ResultType& res) override;

        virtual BT::NodeStatus onFailedRequest(FailureCause failure) override;

        virtual BT::NodeStatus onRunning() override; 
    
        void halt() override;
    
    private:
        Fusion::locked_target *locked_target_; 
        bool is_grasping_; 
};

} // namespace fusion

#endif // GIVE_ACTION_CLIENT_H