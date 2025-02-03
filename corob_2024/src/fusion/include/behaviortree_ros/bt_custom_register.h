
#ifndef CUSTOM_REGISTER_H
#define CUSTOM_REGISTER_H 

#include "behaviortree_ros/bt_action_node.h"
#include "behaviortree_ros/bt_service_node.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include <ros/ros.h>


namespace Fusion {

struct locked_target {
    int target_id; 
    std::string target_name;
    geometry_msgs::PoseWithCovariance pose_msg;
};

}

namespace BT {

// Own version @nmksas
template <class DerivedT> static
  void RegisterRosActionWithStrArg(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle,
                         std::string& str_arg)
{
  NodeBuilder builder = [&node_handle, &str_arg](const std::string& name, const NodeConfig& config) {
    return std::make_unique<DerivedT>(node_handle, name, config, str_arg );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosActionNode< typename DerivedT::ActionType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );
  factory.registerBuilder( manifest, builder );
}

template <class DerivedT> static
  void RegisterRosActionWithTarget(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle,
                         Fusion::locked_target& target)
{
  BT::NodeBuilder builder = [&node_handle, &target](const std::string& name, 
                             const BT::NodeConfig& config) {
    return std::make_unique<DerivedT>(node_handle, name, config, target);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = BT::RosActionNode< typename DerivedT::ActionType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );
  factory.registerBuilder( manifest, builder );
}


// Added custom registration function @nmksas 
template <class DerivedT> static
  void RegisterRosServiceWithStrArg(BT::BehaviorTreeFactory& factory,
                     const std::string& registration_ID,
                     ros::NodeHandle& node_handle,
                     std::string& string_arg)
{
  NodeBuilder builder = [&node_handle, &string_arg](const std::string& name, const NodeConfig& config) {
    return std::make_unique<DerivedT>(node_handle, name, config, string_arg);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosServiceNode< typename DerivedT::ServiceType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}

template <class DerivedT> static
  void RegisterRosServiceWithTarget(BT::BehaviorTreeFactory& factory,
                     const std::string& registration_ID,
                     ros::NodeHandle& node_handle,
                     Fusion::locked_target& target)
{
  NodeBuilder builder = [&node_handle, &target](const std::string& name, 
                         const NodeConfig& config) {
    return std::make_unique<DerivedT>(node_handle, name, config, target);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosServiceNode< typename DerivedT::ServiceType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}



} // namespace BT 

# endif // CUSTOM_REGISTER_H