#ifndef LIST_TARGETS_SERVICE_H
#define LIST_TARGETS_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/ListTargets.h"

namespace Fusion {

class ListTargetsService: 
      public BT::RosServiceNode<fusion::ListTargets> {

    public: 
        ListTargetsService(  ros::NodeHandle& handle, 
                        const std::string& node_name, 
                        const BT::NodeConfig& conf,
                        std::string& action_name);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;

    private: 
        std::string& action_name_; 

};

}

#endif // LIST_TARGETS_SERVICE_H