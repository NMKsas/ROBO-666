#include "fusion_tree.h"

bool const testing = false; 

// replace with your directory 
std::string const TREE_PATH = "/root/ROBO-666/corob_2024/src/fusion/tree_files";
std::string const TREE_NAME = "MainTreeAssembly"; 


namespace Fusion {

FusionTree::FusionTree(BT::BehaviorTreeFactory &factory, 
                       init_config config = DEFAULT_INIT_CONFIG) : 
                       priority_nh_{}, command_nh_{}, spinner_(2), 
                       is_stop_disabled_(true) {
  /**
   * Massive constructor, where an initial state and different nodes are 
   * registered
   */
  this->locked_location_ = DEFAULT_LOCATION;
  this->locked_object_ = DEFAULT_OBJECT;   
  this->spinner_.start(); 


  this->cmd_sub_ = command_nh_.subscribe<std_msgs::String>("/verified_command", 10, &FusionTree::callback, this);
  this->stop_sub_ = priority_nh_.subscribe<std_msgs::Bool>("/stop_enabled", 10, &FusionTree::stopCallback, this); 

  // These are related to state 
  factory.registerSimpleCondition("IsStopCommand",std::bind(&FusionTree::IsStopCommand, this));

  BT::NodeBuilder builder = [this](const std::string& name, const BT::NodeConfig& config){
    return std::make_unique<Fusion::IsCommand>(name, config, this->active_command_); 
  };

  factory.registerNodeType<Fusion::IsAssemblyStarted>("IsAssemblyStarted"); 
  factory.registerBuilder<Fusion::IsCommand>("IsCommand", builder);
  factory.registerNodeType<Fusion::IsGrasping>("IsGrasping"); 
  factory.registerNodeType<Fusion::IsLocked>("IsObjectLocked"); 
  factory.registerNodeType<Fusion::IsLocked>("IsLocationLocked");
  factory.registerNodeType<Fusion::IsTaskDone>("IsTaskDone"); 

  factory.registerSimpleAction("HaltTree", std::bind(&FusionTree::HaltTree, this));
  factory.registerNodeType<BT::LoopNode<uint8_t>>("LoopNode");

  BT::RegisterRosService<Fusion::GripperService>(factory, "CloseGripper", command_nh_); 
  BT::RegisterRosService<Fusion::GripperService>(factory, "OpenGripper", command_nh_); 
  BT::RegisterRosService<Fusion::ConfirmService>(factory, "Confirm", command_nh_); 
  BT::RegisterRosService<Fusion::ConfirmService>(factory, "ConfirmStart", command_nh_); 
  BT::RegisterRosService<Fusion::ConfirmService>(factory, "ConfirmInspection", command_nh_); 
  BT::RegisterRosService<Fusion::ConfirmTargetService>(factory, "ConfirmObject", command_nh_);
  BT::RegisterRosService<Fusion::ConfirmTargetService>(factory, "ConfirmLocation", command_nh_);
  BT::RegisterRosService<Fusion::FeedbackService>(factory, "GiveFeedback", command_nh_);
  BT::RegisterRosService<Fusion::GetTaskToolsService>(factory, "GetTools", command_nh_); 
  BT::RegisterRosService<Fusion::IsFaultService>(factory, "IsFault", command_nh_);
  BT::RegisterRosService<Fusion::TaskInspectionService>(factory, "InspectTask", command_nh_); 
  BT::RegisterRosService<Fusion::IsObjectDetectedService>(factory, "IsObjectDetected", command_nh_); 
  BT::RegisterRosServiceWithTarget<Fusion::SnapToToolService>(factory, "SnapToTool", command_nh_,this->locked_object_); 
  BT::RegisterRosServiceWithStrArg<Fusion::ListTargetsService>(factory, "ListTargets", command_nh_, this->active_command_); 
  BT::RegisterRosService<Fusion::ListToolsService>(factory, "ListTools", command_nh_); 
  BT::RegisterRosService<Fusion::ResetProgressService>(factory, "ResetTaskProgress", command_nh_); 
  BT::RegisterRosService<Fusion::StopService>(factory, "Stop", command_nh_); 
  BT::RegisterRosService<Fusion::TaskInstructionsService>(factory, "GetTaskInstructions", command_nh_); 
  BT::RegisterRosService<Fusion::TaskProgressService>(factory, "GetTaskProgress", command_nh_); 
  BT::RegisterRosService<Fusion::VerifyTaskService>(factory, "VerifyTask", command_nh_); 

  BT::RegisterRosAction<Fusion::MoveActionClient>(factory, "MoveActionCall", command_nh_); 
  BT::RegisterRosActionWithTarget<Fusion::PickActionClient>(factory, "PickActionCall", command_nh_, this->locked_object_); 
  BT::RegisterRosActionWithTarget<Fusion::PlaceActionClient>(factory, "PlaceActionCall", command_nh_, this->locked_location_); 
  BT::RegisterRosActionWithTarget<Fusion::GiveActionClient>(factory, "GiveActionCall", command_nh_, this->locked_location_); 
  BT::RegisterRosActionWithTarget<Fusion::SnapToTargetClient>(factory, "SnapToDirectionVoiceClient", command_nh_, this->locked_location_);
  BT::RegisterRosActionWithTarget<Fusion::SnapToTargetClient>(factory, "SnapToLocationVoiceClient", command_nh_, this->locked_location_);
  BT::RegisterRosActionWithTarget<Fusion::SnapToTargetClient>(factory, "SnapToObjectVoiceClient", command_nh_, this->locked_object_);
  BT::RegisterRosActionWithTarget<Fusion::SnapToTargetClient>(factory, "SnapToLocationGestureClient", command_nh_, this->locked_location_);
  BT::RegisterRosActionWithTarget<Fusion::SnapToTargetClient>(factory, "SnapToObjectGestureClient", command_nh_, this->locked_object_);
  factory.registerSimpleAction("Idle", std::bind(&FusionTree::Idle, this));   
  
  // fetch the tree from file 
  using namespace boost::filesystem; 
  
  for (directory_entry& entry : directory_iterator(TREE_PATH)) {
    if( entry.path().extension() == ".xml"){
      factory.registerBehaviorTreeFromFile(entry.path().string());
    }
  }
  auto global_bb = BT::Blackboard::create(); 
  auto maintree_bb = BT::Blackboard::create(global_bb); 
  this->fusion_tree_ = factory.createTree(config.tree_name, maintree_bb); 
  this->active_command_ = "idle";  
  // set grasping status to false 
  global_bb->set("grasping_status", false); // note that some of the nodes have init in the constructor. 
  global_bb->set("is_assembly_started", false);
  global_bb->set("object_target_id",-1);
  global_bb->set("location_target_id",-1);
  global_bb->set("direction_target_id",-1);
  //global_bb->set("")

  if (testing){
    ros::spinOnce(); 
    return; 
  }
  std::cout << "Constructor end" << std::endl; 
}

FusionTree::~FusionTree() {
  /**
   * Stop the AsyncSpinner
   */
    this->spinner_.stop();   
}


BT::NodeStatus FusionTree::HaltTree() {
  /**
   * Halt and kill tree completely; temporary means to end execution 
   */
    std::cout << "[ Stop service not working. Exiting... ]" << std::endl;
    this->fusion_tree_.haltTree(); 
    this->~FusionTree(); 
    return BT::NodeStatus::FAILURE; 
}

BT::NodeStatus FusionTree::IsStopCommand() {
  /**
   * Evaluate the stop state 
   */
    if ( this->is_stop_disabled_.load() == false){
      std::cout << "[ Stop command received ]" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      std::cout << "[ Regular command received ]" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus FusionTree::Idle() {
  /**
   * Proceed to idle state 
   */
    active_command_ = "idle";
    std::cout << "[ I am idling, sir. ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

void FusionTree::Tick() {
  /**
   * Commences traversing the tree, until SUCCESS or FAILURE status is reached
   */

  BT::NodeStatus status = BT::NodeStatus::IDLE;

  ros::Duration sleep_time(0.01);
  while(ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)){
    status = this->fusion_tree_.tickExactlyOnce();
    std::cout << status << std::endl; 
    sleep_time.sleep();
  }

  // for now, reset robot state to idle and release stop when terminal state is 
  // reached 
  std::cout << "Reset robot to idle state" << std::endl; 
  active_command_ = "idle"; 
  this->is_stop_disabled_.store(true); 

}

void FusionTree::callback(const std_msgs::String::ConstPtr &msg) {
  /**
   * Callback for incoming, verified commands 
   */

  if (active_command_ == "idle") {
    std::cout << "[ Robot is idle and stop is cleared, proceed to tick ]" << std::endl; 
    this->active_command_ = msg->data.c_str(); 
    this->Tick();  
  } 
  std::cout << "[ Other command is still processing ]" << std::endl;
  return;
}

void FusionTree::stopCallback(const std_msgs::Bool::ConstPtr &msg) {
  /**
   * Callback for incoming stop command 
   * */
  this->is_stop_disabled_.store(!msg->data);   
}

} // namespace Fusion