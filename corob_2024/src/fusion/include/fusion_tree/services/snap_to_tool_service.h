#ifndef SNAP_TO_TOOL_SERVICE_H
#define SNAP_TO_TOOL_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include "behaviortree_ros/bt_custom_register.h"
#include <ros/ros.h>
#include "fusion/SnapToTool.h"

namespace Fusion {

class SnapToToolService: 
      public BT::RosServiceNode<fusion::SnapToTool> {

    public: 
        SnapToToolService(ros::NodeHandle& handle, 
                        const std::string& node_name, 
                        const BT::NodeConfig & conf,
                        locked_target& locked_target);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;

    private:
        Fusion::locked_target* locked_target_; 
};

}

#endif // SNAP_TO_TOOL_SERVICE_H