#ifndef RESET_PROGRESS_SERVICE_H 
#define RESET_PROGRESS_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/ResetProgress.h"

namespace Fusion {

class ResetProgressService: 
      public BT::RosServiceNode<fusion::ResetProgress> {

    public: 
        ResetProgressService(ros::NodeHandle& handle, 
                       const std::string& node_name, 
                       const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
};

}

#endif // RESET_PROGRESS_SERVICE_H