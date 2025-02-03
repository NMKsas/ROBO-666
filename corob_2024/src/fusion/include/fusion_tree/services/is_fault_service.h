#ifndef IS_FAULT_SERVICE_H
#define IS_FAULT_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/IsFault.h"

namespace Fusion {

class IsFaultService: 
      public BT::RosServiceNode<fusion::IsFault> {

    public: 
        IsFaultService(ros::NodeHandle& handle, 
                        const std::string& node_name, 
                        const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
};

}

#endif // IS_FAULT_SERVICE_H