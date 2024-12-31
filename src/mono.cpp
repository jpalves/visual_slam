/*

*/
#include <condition_variable>
#include <iostream>
#include <iostream>
#include <queue>
#include <thread>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "System.h"

class VisualSLAM : public rclcpp::Node {
	private:
		sensor_msgs::msg::CameraInfo camera_info;
		ORB_SLAM3::System* mpSLAM_;
		Sophus::SE3f Tcw_SE3F;
		tf2_ros::TransformBroadcaster odom_broadcaster;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
	    rclcpp::Time msg_time;
		//sensor_msgs::msg::CameraInfo camera_info;

	public:
    	VisualSLAM(ORB_SLAM3::System* pSLAM): Node("visual_slam"), mpSLAM_(pSLAM), odom_broadcaster(this){
      		RCLCPP_INFO(this->get_logger(), "Visual SLAM node has been started.");
			
			odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
			tracked_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_in", rclcpp::QoS(10));
    	}

		void GrabImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
			cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
			cv::GaussianBlur(image, image, cv::Size(3, 3), 1, 0);

			//cv::cvtColor(img, image, cv::COLOR_BGR2GRAY);
			msg_time = msg->header.stamp;//bronca
			Tcw_SE3F = mpSLAM_->TrackMonocular(image, msg_time.seconds());// + msg->header.stamp.nanosec*1e-9);
			publish_odometry(Tcw_SE3F, msg_time);
			publish_points(mpSLAM_->GetTrackedMapPoints(), msg_time);
			//publish_points(mpSLAM_->GetAllMapPoints(), msg_time);
    	}

		tf2::Transform TransformFromMat (cv::Mat position_mat) {
    		cv::Mat rotation(3, 3, CV_32F);
    		cv::Mat translation(3, 1, CV_32F);

    		rotation = position_mat.rowRange(0, 3).colRange(0, 3);
    		translation = position_mat.rowRange(0, 3).col(3);


    		tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0, 0),
                                       rotation.at<float> (0, 1),
                                       rotation.at<float> (0, 2),
                                       rotation.at<float> (1, 0),
                                       rotation.at<float> (1, 1),
                                       rotation.at<float> (1, 2),
                                       rotation.at<float> (2, 0),
                                       rotation.at<float> (2, 1),
                                       rotation.at<float> (2, 2));

    		tf2::Vector3 tf_camera_translation (translation.at<float> (0),
                                        translation.at<float> (1),
                                        translation.at<float> (2));

    		//Coordinate transformation matrix from orb coordinate system to ros coordinate system
    		const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                       		   -1, 0, 0,
                                                0, -1, 0);

    		//Transform from orb coordinate system to ros coordinate system on camera coordinates
    		tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    		tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    		//Inverse matrix
    		tf_camera_rotation = tf_camera_rotation.transpose();
    		tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    		//Transform from orb coordinate system to ros coordinate system on map coordinates
    		tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    		tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    		return tf2::Transform (tf_camera_rotation, tf_camera_translation);
		}

		void publish_odometry(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time){
    		cv::Mat Tcw;
			Eigen::Matrix4f TcwE3f = Tcw_SE3f.matrix();
		    cv::eigen2cv(TcwE3f, Tcw);
			tf2::Transform tf_transform = TransformFromMat(Tcw);
			nav_msgs::msg::Odometry odom_msg;
    		
			odom_msg.header.stamp = msg_time;
    		odom_msg.header.frame_id = "odom"; 
    		odom_msg.child_frame_id = "base_footprint";

    		odom_msg.pose.pose.position.x = tf_transform.getOrigin().getX();
    		odom_msg.pose.pose.position.y = tf_transform.getOrigin().getY();
    		odom_msg.pose.pose.position.z = tf_transform.getOrigin().getZ();

    		odom_msg.pose.pose.orientation.x = tf_transform.getRotation().getX();
    		odom_msg.pose.pose.orientation.y = tf_transform.getRotation().getY();
    		odom_msg.pose.pose.orientation.z = tf_transform.getRotation().getZ();
    		odom_msg.pose.pose.orientation.w = tf_transform.getRotation().getW();

    		// Odom
    		odom_pub->publish(odom_msg);
            
    		geometry_msgs::msg::TransformStamped odom_trans;
    		odom_trans.header.stamp = msg_time;
    		odom_trans.header.frame_id = "odom";
    		odom_trans.child_frame_id  = "base_footprint";

    		odom_trans.transform.translation.x = tf_transform.getOrigin().getX();
    		odom_trans.transform.translation.y = tf_transform.getOrigin().getY();
    		odom_trans.transform.translation.z = tf_transform.getOrigin().getZ();
    		odom_trans.transform.rotation.x = tf_transform.getRotation().getX();
    		odom_trans.transform.rotation.y = tf_transform.getRotation().getY();
    		odom_trans.transform.rotation.z = tf_transform.getRotation().getZ();
    		odom_trans.transform.rotation.w = tf_transform.getRotation().getW();
    		odom_broadcaster.sendTransform(odom_trans);
		}

		sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, const rclcpp::Time& msg_time){
			const int num_channels = 3;
    		sensor_msgs::msg::PointCloud2 cloud;

    		cloud.header.stamp = msg_time;
    		cloud.header.frame_id = "odom";
    		cloud.height = 1;
    		cloud.width = map_points.size();
    		cloud.is_bigendian = false;
    		cloud.is_dense = true;
    		cloud.point_step = num_channels * sizeof(float);
    		cloud.row_step = cloud.point_step * cloud.width;
    		cloud.fields.resize(num_channels);

			std::string channel_id[] = { "x", "y", "z"};
    		for (int i = 0; i < num_channels; i++) {
        		cloud.fields[i].name = channel_id[i];
        		cloud.fields[i].offset = i * sizeof(float);
        		cloud.fields[i].count = 1;
        		cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    		}

    		/*for (int i = 0; i < num_channels; ++i) {
        		cloud.fields[i].name = (i == 0) ? "x" : (i == 1) ? "y" : "z";
        		cloud.fields[i].offset = i * sizeof(float);
        		cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        		cloud.fields[i].count = 1;
    		}*/

    		cloud.data.resize(cloud.width * cloud.point_step);
    		//cloud.data.resize(cloud.row_step * cloud.height);

			unsigned char* ptr = cloud.data.data();
			//creio que isto não está correto
    		for (size_t i = 0; i < map_points.size(); ++i) {
        		ORB_SLAM3::MapPoint* pMP = map_points[i];
        		if (pMP) {
            		float* p = reinterpret_cast<float*>(ptr);
            		p[0] = (float)  pMP->GetWorldPos()(2);
            		p[1] = (float) -pMP->GetWorldPos()(0);
            		p[2] = (float) -pMP->GetWorldPos()(1);
            		ptr += cloud.point_step;
        		}
    		}
    		return cloud;
		}	

		void publish_points(const std::vector<ORB_SLAM3::MapPoint*>& points, const rclcpp::Time& msg_time){
    		sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(points, msg_time);
    		
			tracked_mappoints_pub->publish(cloud);
		}

  		~VisualSLAM(){}
};

int main(int argc, char ** argv){
	rclcpp::init(argc, argv);
  	std::string image_topic_= "/camera/camera/color/image_raw/compressed",
				//camera_info_topic_ = "/camera//camera_info",
				config_file_ = "/home/jpalves/ORB_SLAM3/Examples/Monocular/RealSense_D435i.yaml",
				vocabulary_file_ = "/home/jpalves/ORB_SLAM3/Vocabulary/ORBvoc.txt";
				
	ORB_SLAM3::System SLAM(vocabulary_file_, config_file_, ORB_SLAM3::System::MONOCULAR, true);

	auto node = std::make_shared<VisualSLAM>(&SLAM);
  	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img =
            node->create_subscription<sensor_msgs::msg::CompressedImage>(
            image_topic_, 1, std::bind(&VisualSLAM::GrabImage, node, std::placeholders::_1));
    
	//sensor_msgs::msg::CameraInfo camera_info;

	//auto ret = rclcpp::wait_for_message(camera_info, node, camera_info_topic_);
	/*if(ret){
		RCLCPP_INFO(node->get_logger(), "Camera info received.");
	}*/

  	rclcpp::spin(node);
  	rclcpp::shutdown();
  
  	return 0;
}
