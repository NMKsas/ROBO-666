#include <iostream> 
#include <boost/filesystem.hpp> 
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <thread> 
#include <mutex>
#include <atomic> 

class TestClass{
    public: 
        TestClass();
        ~TestClass();
        void tick();
        void regularCallback(const std_msgs::String::ConstPtr& msg); 
        void priorityCallback(const std_msgs::Bool::ConstPtr& msg); 

    private: 
        bool is_stop = false; 
        std::atomic<bool> stop_triggered; 
        std::string received_command;  
        std::string active_command; 
        std::mutex mutex_; 
        std::string status; 
        ros::AsyncSpinner spinner_; 
        ros::CallbackQueue priority_queue_;
        ros::NodeHandle command_nh_; 
        ros::NodeHandle priority_nh_; 
        ros::Subscriber cmd_sub_; 
        ros::Subscriber stop_sub_;
};