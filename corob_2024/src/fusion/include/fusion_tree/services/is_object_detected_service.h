#ifndef IS_OBJECT_DETECTED_SERVICE_H
#define IS_OBJECT_DETECTED_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/IsObjectDetected.h"

namespace Fusion {

class IsObjectDetectedService: 
      public BT::RosServiceNode<fusion::IsObjectDetected> {

    public: 
        IsObjectDetectedService(ros::NodeHandle& handle, 
                        const std::string& node_name, 
                        const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
};

}

#endif // IS_OBJECT_DETECTED_SERVICE_H