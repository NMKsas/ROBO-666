#ifndef TASK_INSTRUCTIONS_SERVICE_H
#define TASK_INSTRUCTIONS_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/TaskInstructions.h"

namespace Fusion {

class TaskInstructionsService: 
      public BT::RosServiceNode<fusion::TaskInstructions> {

    public: 
        TaskInstructionsService(ros::NodeHandle& handle, 
                       const std::string& node_name, 
                       const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
};

}

#endif // TASK_INSTRUCTIONS_SERVICE_H