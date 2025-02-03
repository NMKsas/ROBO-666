#include <gtest/gtest.h>
#include <ros/ros.h>
#include "fusion_tree/fusion_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/String.h"
#include <chrono>
#include <unordered_set>

using namespace BT;
using namespace Fusion; 

/*
struct state_booleans {
    bool is_grasping_;          #1 grasping
    bool is_object_locked_;     #2 object locked
    bool is_stop_disabled_;     #4 stop disabled 
    bool is_location_locked_;   #5 location locked 
}; 
*/

static const int NO_OBJECT_DETECTED = 0; 

// The fixture for testing class FusionTreeTest
class FusionPickSubtreeTest : public testing::Test {
 protected:

    // initial active command
    init_config initObjectGraspedNoneDetected; 
    init_config initNoneGraspedObjectDetected;
    init_config initNoneGraspedNoneDetected; 
    init_config initNoInitialGuess; 
    init_config initInitialGuess; 

    // grasped, locked, stop cmd disabled 
    state_booleans state_1_1 = {true, false, true, false}; 
    state_booleans state_1_2 = {false, true, true, false};
    state_booleans state_2_1 = {false, false, true, false}; 
    state_booleans state_3_1 = {false, false, true, false};
    state_booleans state_3_2 = {false, true, true, false};

    std::unordered_set<std::string> empty_object_set = {};
    std::unordered_set<std::string> detected_set = {"ALLEN KEY", "BOLT"}; 

    FusionPickSubtreeTest() {
        
        // init active commands 
        initObjectGraspedNoneDetected = { "pick up", "PickSubtree", state_1_1, empty_object_set};
        initNoneGraspedObjectDetected = { "pick up", "PickSubtree", state_1_2, detected_set}; 
        initNoneGraspedNoneDetected = { "pick up", "PickSubtree", state_2_1, empty_object_set};
        initNoInitialGuess = {"pick up", "PickSubtree", state_3_1, detected_set}; 
        initInitialGuess = {"pick up", "PickSubtree", state_3_2, detected_set}; 
    }
    ~FusionPickSubtreeTest() override {
        ros::shutdown(); 
    }

};


/**
 * TEST SUITE 2: PICK-SUBTREE TESTING 
 */

/**
 * TEST 1: Grasping state 
 */

/**
 * TEST 1.1: If robot is grasping something, the object is placed 
 *           before initializing the pick up action 
 *      Prerequisites: - grasping state can be polled from the robot
 *                     - place subtree exists
 *                     - default area for placing is defined 
 *      Expected: Tree traverses to execute Place subtree      
 */

TEST_F(FusionPickSubtreeTest, graspedObjectNoneDetected) {
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory,
                                          initObjectGraspedNoneDetected); 
    fusion_node->Tick(); 
    std::vector<std::string> actionList = fusion_node->getTraverseHistory();

    if (actionList.empty()){
        FAIL() << "The action list shouldn't be empty"; 
    }
    
    std::string last_action = actionList.back();
    std::string expected_last_action = "interrupt"; 
    int interrupt_reason = fusion_node->getInterruptReason(); 
    delete fusion_node; 

    EXPECT_EQ(interrupt_reason, NO_OBJECT_DETECTED);
    EXPECT_EQ(last_action,expected_last_action);

}

/**
 * TEST 1.2: If no object is grasped, tree traverses forward. 
 *      Prerequisites: - grasping state can be polled from the robot 
 *      Expected: Tree traverses to precondition check 
 */   

TEST_F(FusionPickSubtreeTest, noGraspingObjectDetected){
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initNoneGraspedObjectDetected); 
    fusion_node->Tick(); 
    std::vector<std::string> actionList = fusion_node->getTraverseHistory();

    if (actionList.empty()){
        FAIL() << "The action list shouldn't be empty"; 
    }
    
    std::string action = actionList.back();
    std::string expected_action = "pick up";
    delete fusion_node; 

    EXPECT_EQ(action,expected_action);
}


/**
 * TEST 2: Object detection
 */

/**
 * TEST 2.1: Picking cannot be executed if no object is seen.
 *      Prerequisites: - state has a list of identified objects 
 *      Expected: Pick subtree fails, command is discarded 
 */

TEST_F(FusionPickSubtreeTest, noObjectDetected){
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initNoneGraspedNoneDetected); 
    fusion_node->Tick(); 
    std::vector<std::string> actionList = fusion_node->getTraverseHistory();

    if (actionList.empty()){
        FAIL() << "The action list shouldn't be empty"; 
    }
    
    std::string action = actionList.back();
    std::string expected_action = "interrupt";
    delete fusion_node; 

    EXPECT_EQ(action,expected_action);
}


/**
 * TEST 2.2: Feedback is given if no object is detected.
 *      Prerequisites: - 
 *      Expected: Feedback is given to the end-user
 *      TODO: Test if feature is more robust than console log. 
 */


/**
 * TEST 3. Initial object guess 
 */

/**
 * TEST 3.1: If there is no explicit initial guess, lock subtree is executed.
 *      Expected: Tree traverses to find target 
 *      TODO: Change when initial guess exists as an actual feature 
 */

TEST_F(FusionPickSubtreeTest, noInitialGuess){
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initNoInitialGuess); 
    fusion_node->Tick(); 
    std::vector<std::string> actionList = fusion_node->getTraverseHistory();

    if (std::find(actionList.begin(), actionList.end(), "lock to location") != actionList.end()){
        delete fusion_node; 
        SUCCEED() << "Query for locking to location done";  
    }
}

/**
 * TEST 3.2: If an explicit initial guess exists, the object is picked directly. 
 *      Prerequisites: - the user has provided robot an initial guess (one way
 *                       or another)
 *      Expected: Pick action call is performed right after the initial guess is
 *                confirmed
 *      TODO: Change when initial guess exists as an actual feature 
 */


TEST_F(FusionPickSubtreeTest, initialGuess){
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initInitialGuess); 
    fusion_node->Tick(); 
    std::vector<std::string> actionList = fusion_node->getTraverseHistory();

    if (actionList.empty()){
        FAIL() << "The action list shouldn't be empty"; 
    }
    
    std::string last_action = actionList.back();
    std::string expected_last_action = "pick up"; 
    delete fusion_node; 

    EXPECT_EQ(last_action,expected_last_action);
}

/**
 * TEST 4: Successful locking  
 */

/**
 * TEST 4.1: Feedback is given, when the locking is complete. 
 *      Expected: Feedback is given to the end-user 
 *      TODO: Test if feature is more robust than console log. 
 */

/**
 * TEST 4.2: Completed locking results in pick object action call. 
 *      Expected: Locked object state results in pick action call
 *      
 *      See testcase 3.1. 
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "test_pick");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
