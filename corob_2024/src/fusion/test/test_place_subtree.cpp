#include <gtest/gtest.h>
#include <ros/ros.h>
#include "fusion_tree/fusion_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/String.h"
#include <algorithm>
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

// The fixture for testing class FusionTreeTest
class FusionPlaceSubtreeTest : public testing::Test {
 protected:

    // initial active command
    init_config initObjectGrasped; 
    init_config initObjectGraspedLockedLocation;
    init_config initObjectNotGrasped; 

    // grasped, obj. locked, stop cmd disabled, loc. locked
    state_booleans state_1_1 = {true, true, true, false}; 
    state_booleans state_1_2 = {true, true, true, true};
    state_booleans state_1_3 = {false, false, true, false}; 

    std::unordered_set<std::string> detected_set = {"ALLEN KEY", "BOLT"}; 

    FusionPlaceSubtreeTest() {
        
        // init active commands 
        initObjectGrasped = { "place", "PlaceSubtree", state_1_1, detected_set};
        initObjectGraspedLockedLocation = { "place", "PlaceSubtree", state_1_2, detected_set}; 
        initObjectNotGrasped = { "place", "PlaceSubtree", state_1_3, detected_set};
    }
    ~FusionPlaceSubtreeTest() override {
        ros::shutdown(); 
    }

};


/**
 * TEST SUITE 3: PLACE-SUBTREE TESTING 
 */

/**
 * TEST 1: Grasping state 
 */

/**
 * TEST 1.1: If robot is grasping something, the location is immediately 
 *           queried.
 * Prerequisite: - grasp status is published, and can be queried 
 */


TEST_F(FusionPlaceSubtreeTest, objectGrasped){
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initObjectGrasped); 
    fusion_node->Tick(); 

    std::vector<std::string> actionList = fusion_node->getTraverseHistory();

    if (actionList.size() < 2){
        FAIL() << "The action list should contain at least two items"; 
    }
    if (std::find(actionList.begin(), actionList.end(), "pick up") != actionList.end()){
        FAIL() << "Performed pick up branch, when already grasping object"; 
    } 
    if (std::find(actionList.begin(), actionList.end(), "lock to location") != actionList.end()){
        delete fusion_node; 
        SUCCEED() << "Query for locking to location done";  
    }
}

/**
 * TEST 1.2: If robot is grasping and the location is locked, place action is
 *           directly executed.
 * Prerequisite: - grasp status is published, and can be queried 
 */

TEST_F(FusionPlaceSubtreeTest, objectGraspedLockedLocation){
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initObjectGraspedLockedLocation); 
    fusion_node->Tick(); 
    std::vector<std::string> actionList = fusion_node->getTraverseHistory();

    if (std::find(actionList.begin(), actionList.end(), "lock to location") != actionList.end()){
        delete fusion_node; 
        FAIL() << "No locking needed";  
    }

}

/**
 * TEST 1.3: If robot is not grasping anything, pick subtree is executed.
 */


TEST_F(FusionPlaceSubtreeTest, objectNotGrasped){
    BT::BehaviorTreeFactory factory; 
    Fusion::FusionTree* fusion_node = new Fusion::FusionTree(factory, initObjectNotGrasped); 
    fusion_node->Tick(); 
    std::vector<std::string> actionList = fusion_node->getTraverseHistory();

    if (std::find(actionList.begin(), actionList.end(), "pick up") != actionList.end()){
        delete fusion_node; 
        SUCCEED() << "Pick up branch has been ticked"; 
    }

}

/**
 * TEST 2:  
 */

/**
 * TEST 2.1: If there is no explicit location guess, lock subtree is executed.
 *      Expected: Tree traverses to find target 
 *      TODO: Change when initial guess exists as an actual feature 
 * 
 *      See test 1.1.
 */

/**
 * TEST 2.2: If an explicit location guess exists, the object is placed directly. 
 *      Prerequisites: - the user has provided robot an initial guess (one way
 *                       or another)
 *      Expected: Pick action call is performed right after the initial guess is
 *                confirmed
 *      TODO: Change when initial guess exists as an actual feature 
 * 
 *      See test 1.2.
 */


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_place");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}