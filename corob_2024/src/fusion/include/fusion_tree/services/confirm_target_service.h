#ifndef CONFIRM_TARGET_SERVICE_H
#define CONFIRM_TARGET_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/ConfirmTarget.h"

namespace Fusion {

class ConfirmTargetService: 
      public BT::RosServiceNode<fusion::ConfirmTarget> {

    public: 
        ConfirmTargetService(ros::NodeHandle& handle, 
                             const std::string& node_name, 
                             const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
};

}

#endif // CONFIRM_TARGET_SERVICE_H