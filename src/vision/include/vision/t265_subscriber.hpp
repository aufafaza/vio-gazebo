#pragma once 
#include "rclcpp/rclcpp.hpp"  
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "nav_msgs/msg/odometry.hpp" 
#include "mavconn/interface.hpp" 
#include "tf2_ros/tf2_ros/transform_broadcaster.hpp"
class T265Subscriber : public rclcpp::Node{ 
public: 

	T265Subscriber() : rclcpp::Node("t265_pose_subscriber") 
	{

	fcu_ = mavconn::MAVConnInterface::open_url(
	    "udp://127.0.0.1:14550@", 1, mavconn::MAV_COMP_ID_UDP_BRIDGE,
	    [](const mavlink::mavlink_message_t * msg, const mavconn::Framing framing) {
	    });
		odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/camera/pose/sample", 10, 
		std::bind(&T265Subscriber::odom_callback, this, std::placeholders::_1));
	}
private:
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; 
	mavconn::MAVConnInterface::Ptr fcu_; 
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

}; 
