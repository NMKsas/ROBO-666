#include "is_command.h"


namespace Fusion {

IsCommand::IsCommand(const std::string &name, 
const BT::NodeConfig &config, 
std::string &command) : SyncActionNode(name, config), command(&command)
{
}

BT::PortsList IsCommand::providedPorts()
{
    return { BT::InputPort<std::string>("command")};
}

BT::NodeStatus IsCommand::tick() {

    auto command_in = getInput<std::string>("command");

    if (command_in.value() == *command){
        return BT::NodeStatus::SUCCESS; 
    }
    return BT::NodeStatus::FAILURE; 
}

} // namespace Fusion 
