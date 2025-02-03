#ifndef LIST_TOOLS_SERVICE_H
#define LIST_TOOLS_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/ListTools.h"

namespace Fusion {

class ListToolsService: 
      public BT::RosServiceNode<fusion::ListTools> {

    public: 
        ListToolsService(  ros::NodeHandle& handle, 
                        const std::string& node_name, 
                        const BT::NodeConfig& conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;

};

}

#endif // LIST_TOOLS_SERVICE_H