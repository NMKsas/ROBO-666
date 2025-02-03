#ifndef STOP_SERVICE_H 
#define STOP_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/ControllerStop.h"

namespace Fusion {

class StopService: 
      public BT::RosServiceNode<fusion::ControllerStop> {

    public: 
        StopService(ros::NodeHandle& handle, 
                        const std::string& node_name, 
                        const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
};

}

#endif // STOP_SERVICE_H