#ifndef CONFIRM_SERVICE_H
#define CONFIRM_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/Confirm.h"

namespace Fusion {

class ConfirmService: 
      public BT::RosServiceNode<fusion::Confirm> {

    public: 
        ConfirmService(ros::NodeHandle& handle, 
                       const std::string& node_name, 
                       const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
    private: 
      int num_attempts_; 
};

}

#endif // CONFIRM_SERVICE_H