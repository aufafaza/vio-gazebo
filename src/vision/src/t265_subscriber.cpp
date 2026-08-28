#include "vision/t265_subscriber.hpp" 
#include "mavlink/v2.0/common/mavlink_msg_vision_position_estimate.hpp"
#include "tf2_ros/tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp" 
void T265Subscriber::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(), "Position x->%.2f, y->%.2f, z->%.2f", 
	     msg->pose.pose.position.x,
	     msg->pose.pose.position.y, 
	     msg->pose.pose.position.z
	     );

	mavlink::common::msg::VISION_POSITION_ESTIMATE vpe{}; 
	vpe.usec = this->now().nanoseconds() / 1000; 
	vpe.x     = msg->pose.pose.position.x;	
	vpe.y     = msg->pose.pose.position.y;
	vpe.z     = msg->pose.pose.position.z;
	vpe.roll  = 0.0f;
	vpe.pitch = 0.0f;
	vpe.yaw   = 0.0f;
	vpe.reset_counter = 0;
	fcu_->send_message(vpe);
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = msg->header.stamp; 
	t.header.frame_id = "world";
	t.child_frame_id = "camera_pose_frame";
	t.transform.translation.x = msg->pose.pose.position.x;
	t.transform.translation.y = msg->pose.pose.position.y;
	t.transform.translation.z = msg->pose.pose.position.z;
	t.transform.rotation = msg->pose.pose.orientation;
	tf_broadcaster_->sendTransform(t);
}

int main(int argc, char** argv) { 
	rclcpp::init(argc, argv); 
	rclcpp::spin(std::make_shared<T265Subscriber>()); 
	rclcpp::shutdown(); 
	return 0;
}
