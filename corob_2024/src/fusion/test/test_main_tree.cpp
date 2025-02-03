#include <gtest/gtest.h>
#include <ros/ros.h>
#include "fusion_tree/fusion_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/String.h"
#include <chrono>

using namespace BT;
using namespace Fusion; 

// The fixture for testing class FusionTreeTest
class FusionTreeTest : public testing::Test {
 protected:

    // initial active command
    init_config initIdle; 
    init_config initPick;
    init_config initPlace; 
    init_config initStop; 

    // mockup messages for commands 
    std_msgs::String::ConstPtr unknownCmd;
    std_msgs::String::ConstPtr pickCmd;
    std_msgs::String::ConstPtr placeCmd;
    std_msgs::String::ConstPtr stopCmd;
    std_msgs::String::ConstPtr giveCmd;

    FusionTreeTest() {
        
        // init active commands 
        initIdle = { "idle", "MainTree", DEFAULT_STATE_FLAGS, OBJECT_SET }; 
        initPick = { "pick up", "MainTree", DEFAULT_STATE_FLAGS, OBJECT_SET };
        initPlace = { "place", "MainTree", DEFAULT_STATE_FLAGS, OBJECT_SET };
        initStop = { "stop", "MainTree", DEFAULT_STATE_FLAGS, OBJECT_SET }; 

        // define messages 
        std_msgs::String msg; 
        msg.data = "run"; 
        unknownCmd = std_msgs::String::ConstPtr(new std_msgs::String(msg));
        msg.data = "pick up"; 
        pickCmd = std_msgs::String::ConstPtr(new std_msgs::String(msg));
        msg.data = "place"; 
        placeCmd = std_msgs::String::ConstPtr(new std_msgs::String(msg));
        msg.data = "stop"; 
        stopCmd = std_msgs::String::ConstPtr(new std_msgs::String(msg));
        msg.data = "give";  
        giveCmd = std_msgs::String::ConstPtr(new std_msgs::String(msg));
    }
    ~FusionTreeTest() override {
        ros::shutdown(); 
    }

};


/**
 * TEST SUITE 1: MAIN LOGIC 
 */

/**
 * TEST 1: A new command is given, when robot is idle
 * 
 * Prerequisite: Robot is in idle state
 * Expected: Robot state transitions from idle to ACTIVE 
 */

/**
 * TEST 1.1: New command is a stop command
 * 
 * Expected: stop command is executed and the traversing tree ends  
 */


TEST_F(FusionTreeTest, stopCommandWhileIdle) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initIdle); 

    fusion_node->callback(stopCmd); 

    std::vector<std::string> actionList = fusion_node->getTraverseHistory();

    if (actionList.size() < 1){
        FAIL() << "The action list should contain more than one item"; 
    }
    
    std::string expected_active_cmd = "stop";
    std::string active_cmd = actionList.back();
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}


/**
 * TEST 1.2: New command is a regular command
 *      a) pick up
 *      b) place
 *      c) give
 * Expected: Tree traverses until it reaches the corresponding sub tree
 */

TEST_F(FusionTreeTest, pickCommandWhileIdle) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initIdle); 

    fusion_node->callback(pickCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "pick up";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

TEST_F(FusionTreeTest, placeCommandWhileIdle) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initIdle); 

    fusion_node->callback(placeCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "place";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

TEST_F(FusionTreeTest, giveCommandWhileIdle) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initIdle); 

    fusion_node->callback(giveCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "give";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

/**
 * TEST 2: A new command is given, when robot is processing previous command
 * 
 * Prerequisite: Robot is in ACTIVE state, processing command
 */

/**
 * TEST 2.1: New command is a stop command
 * 
 * Expected: stop command is executed and the traversing tree ends  
 */

TEST_F(FusionTreeTest, stopCommandWhileActive) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initPick); 

    fusion_node->callback(stopCmd); 


    std::vector<std::string> actionList = fusion_node->getTraverseHistory();

    if (actionList.size() < 1){
        FAIL() << "The action list should contain more than one item"; 
    }
    
    std::string expected_active_cmd = "stop";
    std::string active_cmd = actionList.back();
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

/**
 * TEST 2.2: New command is a regular command
 *          a) pick up
 *          b) place
 *          c) give
 * Expected: The command is ignored
 */

TEST_F(FusionTreeTest, pickCommandWhileActive) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initPlace); 

    fusion_node->callback(pickCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "place";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

TEST_F(FusionTreeTest, placeCommandWhileActive) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initPick); 

    fusion_node->callback(placeCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "pick up";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

TEST_F(FusionTreeTest, giveCommandWhileActive) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initPick); 

    fusion_node->callback(giveCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "pick up";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

/**
 * TEST 2.3: No command can override or interrupt stop command; 
 *           stop must be cleared to idle before receiving any commands 
 *           a) stop active, a new stop command sent 
 *           b) stop active, a regular command sent
 * 
 * Expected: If stop command is active, all commands are ignored 
 */

TEST_F(FusionTreeTest, stopCommandWhileStopped) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initStop); 

    fusion_node->callback(stopCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "stop";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

TEST_F(FusionTreeTest, regularCommandWhileStopped) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initStop); 

    fusion_node->callback(pickCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "stop";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}


/**
 * TEST 3: The command is not recognized 
 * 
 * Expected: The command is ignored, feedback given to the user
 *           a) initial status is idle 
 *           b) initial status is stop 
 *           c) initial status is a regular command  
 */

TEST_F(FusionTreeTest, unknownCommandWhileIdle) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initIdle); 

    fusion_node->callback(unknownCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "idle";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

TEST_F(FusionTreeTest, unknownCommandWhileStop) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initStop); 

    fusion_node->callback(unknownCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "stop";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

TEST_F(FusionTreeTest, unknownCommandWhileActive) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initPick); 

    fusion_node->callback(unknownCmd); 

    std::string active_cmd = fusion_node->getActiveCommand(); 
    std::string expected_active_cmd = "pick up";
    delete fusion_node; 

    EXPECT_EQ(active_cmd, expected_active_cmd);
}

/**
 * TEST SUITE 3: INITIALIZATION MODULE 
 */

/**
 * TEST 1: ...
 * 
 */

/**
 * TEST SUITE 4: INTERPRETATION MODULE 
 */

/**
 * TEST 1: ... 
 * 
 */


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_main");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}