#ifndef TASK_PROGRESS_SERVICE_H
#define TASK_PROGRESS_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/TaskProgress.h"

namespace Fusion {

class TaskProgressService: 
      public BT::RosServiceNode<fusion::TaskProgress> {

    public: 
        TaskProgressService(ros::NodeHandle& handle, 
                       const std::string& node_name, 
                       const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
};

}

#endif // TASK_PROGRESS_SERVICE_H