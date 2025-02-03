#ifndef VERIFY_TASK_SERVICE_H 
#define VERIFY_TASK_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/VerifyTask.h"

namespace Fusion {

class VerifyTaskService: 
      public BT::RosServiceNode<fusion::VerifyTask> {

    public: 
        VerifyTaskService(ros::NodeHandle& handle, 
                       const std::string& node_name, 
                       const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
};

}

#endif // VERIFY_TASK_SERVICE_H