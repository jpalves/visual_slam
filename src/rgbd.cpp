/*

*/
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

#include"System.h"

int main(int argc, char **argv){
    std::string config_file_ = "/home/jpalves/ORB_SLAM3/Examples/RGB-D/RealSense_D435i.yaml",
				vocabulary_file_ = "/home/jpalves/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    
    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool visualization = true;
    ORB_SLAM3::System SLAM(vocabulary_file_, config_file_, ORB_SLAM3::System::RGBD, visualization);

    auto node = std::make_shared<RgbdSlamNode>(&SLAM);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}