#ifndef VISION_TO_MAVROS_H
#define VISION_TO_MAVROS_H
#include <array>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <mavros_msgs/msg/landing_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class VisionToMavros : public rclcpp::Node {
public:
  VisionToMavros();
  ~VisionToMavros() {}
  void run(void);

private:
  void navigationParameters(void);
  void precisionLandParameters(void);
  void transformReady(
      const std::shared_future<geometry_msgs::msg::TransformStamped> &);
  bool waitForFirstTransform(double);
  void publishVisionPositionEstimate();
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void detectPoseJump(const tf2::Vector3 &current_pos);
  std::array<double, 36> calculatePoseCovariance(int confidence_level);
  std::array<double, 36> calculateTwistCovariance(int confidence_level);

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      camera_pose_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr body_path_publisher;
  rclcpp::Publisher<mavros_msgs::msg::LandingTarget>::SharedPtr
      precland_msg_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      vision_pose_cov_publisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      vision_speed_publisher;

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

  // TF components
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener;
  geometry_msgs::msg::TransformStamped transform_stamped;

  // Transform variables
  tf2::Vector3 position_orig, position_body;
  tf2::Quaternion quat_cam, quat_cam_to_body, quat_rot_z, quat_body;

  // Messages
  geometry_msgs::msg::PoseStamped msg_body_pose;
  nav_msgs::msg::Path body_path;

  // Velocity and covariance from T265
  tf2::Vector3 velocity_cam;
  std::array<double, 36> velocity_covariance_cam;
  std::array<double, 36> pose_covariance_cam;
  bool velocity_received;
  std::mutex velocity_mutex;

  // Timing
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Time last_tf_time;

  // Frame IDs
  std::string target_frame_id;
  std::string source_frame_id;
  std::string precland_target_frame_id;
  std::string precland_camera_frame_id;
  std::string odom_topic;

  // Parameters
  double output_rate;
  double gamma_world;
  double roll_cam;
  double pitch_cam;
  double yaw_cam;
  bool enable_precland;

  // Covariance and tracking parameters
  double linear_accel_cov;
  double angular_vel_cov;
  double pose_jump_threshold;

  // Tracking variables
  tf2::Vector3 prev_position_body;
  bool first_pose_received;
  uint8_t reset_counter;
};
#endif
