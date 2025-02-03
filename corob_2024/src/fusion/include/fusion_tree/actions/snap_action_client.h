#ifndef SNAP_ACTION_CLIENT_H
#define SNAP_ACTION_CLIENT_H

#include "behaviortree_ros/bt_action_node.h"
#include "behaviortree_ros/bt_custom_register.h"
#include <ros/ros.h>
#include "fusion/SnapAction.h"

namespace Fusion {

class SnapToTargetClient: public BT::RosActionNode<fusion::SnapAction> {

    public:
        SnapToTargetClient( ros::NodeHandle& handle, const std::string& name, 
                            const BT::NodeConfig& conf, 
                            locked_target& locked_target);   

        static BT::PortsList providedPorts();

        bool sendGoal(GoalType& goal) override;

        BT::NodeStatus onResult( const ResultType& res) override;

        virtual BT::NodeStatus onFailedRequest(FailureCause failure) override;
  
        void halt_(); //override;
    
    private:
        Fusion::locked_target* locked_target_; 
};

} // namespace fusion

#endif // SNAP_ACTION_CLIENT_H