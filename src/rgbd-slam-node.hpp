
#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"


class RgbdSlamNode : public rclcpp::Node{
private: 
    using ImageMsg = sensor_msgs::msg::CompressedImage;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage> approximate_sync_policy;

    void GrabRGBD(const sensor_msgs::msg::CompressedImage::SharedPtr msgRGB, const sensor_msgs::msg::CompressedImage::SharedPtr msgD);
    sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, const rclcpp::Time& msg_time);
    void publish_odometry(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time);
    tf2::Transform TransformFromMat (cv::Mat position_mat);
    void publish_points(const std::vector<ORB_SLAM3::MapPoint*>& points, const rclcpp::Time& msg_time);

    tf2_ros::TransformBroadcaster odom_broadcaster;
    ORB_SLAM3::System* m_SLAM;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    //cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;
    Sophus::SE3f Tcw_SE3F;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage> > depth_sub;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

public:
    RgbdSlamNode(ORB_SLAM3::System* pSLAM);
    ~RgbdSlamNode();
};

#endif
