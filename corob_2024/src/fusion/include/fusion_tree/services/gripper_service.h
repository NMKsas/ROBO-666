#ifndef GRIPPER_SERVICE_H
#define GRIPPER_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/ActivateGripper.h"

namespace Fusion {

class GripperService: 
    public BT::RosServiceNode<fusion::ActivateGripper> {

        public: 
            GripperService(ros::NodeHandle& handle, 
                           const std::string& node_name, 
                           const BT::NodeConfig & conf);            

            static BT::PortsList providedPorts();

            void sendRequest(RequestType& request) override;

            BT::NodeStatus onResponse(const ResponseType& rep) override;

        private: 
            bool is_open_; 
};

}

#endif // GRIPPER_SERVICE_H