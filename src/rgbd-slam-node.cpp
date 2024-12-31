#include "rgbd-slam-node.hpp"

//#include<opencv2/core/core.hpp>
//#include <opencv2/core/eigen.hpp>


using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("orbslam"),
    odom_broadcaster(this),
    m_SLAM(pSLAM){
    rgb_sub   = std::make_shared< message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/color/image_raw/compressed");
    depth_sub = std::make_shared< message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/depth/image_rect_raw/compressedDepth");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tracked_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_in", rclcpp::QoS(10));
}

RgbdSlamNode::~RgbdSlamNode(){
    // Stop all threads
    m_SLAM->Shutdown();
}

sensor_msgs::msg::PointCloud2 RgbdSlamNode::mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, const rclcpp::Time& msg_time){
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

    for (int i = 0; i < num_channels; ++i) {
        cloud.fields[i].name = (i == 0) ? "x" : (i == 1) ? "y" : "z";
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count = 1;
    }

    cloud.data.resize(cloud.width * cloud.point_step);
    		
	unsigned char* ptr = cloud.data.data();
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

void RgbdSlamNode::publish_odometry(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time){
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

void RgbdSlamNode::publish_points(const std::vector<ORB_SLAM3::MapPoint*>& points, const rclcpp::Time& msg_time){
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(points, msg_time);
    		
	tracked_mappoints_pub->publish(cloud);
}

tf2::Transform RgbdSlamNode::TransformFromMat(cv::Mat position_mat) {
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
    const tf2::Matrix3x3 tf_orb_to_ros (0,  0, 1,
                                	   -1,  0, 0,
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

//refazer
void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD){
    // Copy the ros rgb image message to cv::Mat.
   
    cv::Mat image = cv::imdecode(cv::Mat(msgRGB->data), cv::IMREAD_COLOR);
    
    
    auto logger = rclcpp::get_logger("compressed_depth_image_transport");
    rclcpp::Time msg_time = this->get_clock()->now(); //msgRGB->header.stamp;
    
    // Get compressed image data
    const std::vector<uint8_t> imageData(msgD->data.begin() + 12, msgD->data.end());
    cv::Mat decompressed;

    try {
          // Decode image data
          decompressed = cv::imdecode(imageData, cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
    } catch (cv::Exception & e) {
          RCLCPP_ERROR(logger, "%s", e.what());
    }
    
    Tcw_SE3F = m_SLAM->TrackRGBD(image, decompressed, this->get_clock()->now().seconds());
    publish_odometry(Tcw_SE3F, msg_time);
    publish_points(m_SLAM->GetTrackedMapPoints(), msg_time);
}
