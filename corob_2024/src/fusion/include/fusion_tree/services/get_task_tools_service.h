#ifndef GET_TASK_TOOLS_H
#define GET_TASK_TOOLS_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/GetTools.h"
#include <deque>

namespace Fusion {

class GetTaskToolsService: 
      public BT::RosServiceNode<fusion::GetTools> {

    public: 
        GetTaskToolsService(ros::NodeHandle& handle, 
                        const std::string& node_name, 
                        const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
        
};

}

#endif // GET_TASK_TOOLS_H