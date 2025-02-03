#ifndef TASK_INSPECTION_SERVICE_H 
#define TASK_INSPECTION_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/Inspection.h"

namespace Fusion {

class TaskInspectionService: 
      public BT::RosServiceNode<fusion::Inspection> {

    public: 
        TaskInspectionService(ros::NodeHandle& handle, 
                       const std::string& node_name, 
                       const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
};

}

#endif // TASK_INSPECTION_SERVICE_H