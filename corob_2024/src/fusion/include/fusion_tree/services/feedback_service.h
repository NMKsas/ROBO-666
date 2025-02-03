#ifndef FEEDBACK_SERVICE_H
#define FEEDBACK_SERVICE_H

#include "behaviortree_ros/bt_service_node.h"
#include <ros/ros.h>
#include "fusion/Feedback.h"

namespace Fusion {

class FeedbackService: 
      public BT::RosServiceNode<fusion::Feedback> {

    public: 
        FeedbackService( ros::NodeHandle& handle,
                          const std::string& node_name, 
                          const BT::NodeConfig & conf);            

        static BT::PortsList providedPorts();

        void sendRequest(RequestType& request) override;

        BT::NodeStatus onResponse(const ResponseType& rep) override;
};

} // namespace Fusion

#endif // FEEDBACK_SERVICE_H