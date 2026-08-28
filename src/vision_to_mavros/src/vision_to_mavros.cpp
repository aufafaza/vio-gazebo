#include <vision_to_mavros/vision_to_mavros.hpp>

VisionToMavros::VisionToMavros()
    : Node("vision_to_mavros_node"), first_pose_received(false),
      reset_counter(1), linear_accel_cov(0.01), angular_vel_cov(0.01),
      pose_jump_threshold(0.1), velocity_received(false) {

  pose_covariance_cam.fill(0.0);
  velocity_covariance_cam.fill(0.0);

  buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  buffer->setCreateTimerInterface(timer_interface);
  transform_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

  this->navigationParameters();
  this->precisionLandParameters();
}

void VisionToMavros::navigationParameters(void) {
  camera_pose_publisher =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("vision_pose",
                                                              10);
  body_path_publisher =
      this->create_publisher<nav_msgs::msg::Path>("body_frame/path", 1);
  vision_pose_cov_publisher =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/mavros/vision_pose/pose_cov", 10);
  vision_speed_publisher =
      this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "/mavros/vision_speed/speed_twist_cov", 10);

  this->declare_parameter<std::string>("target_frame_id", "odom_frame");
  this->get_parameter("target_frame_id", target_frame_id);
  this->declare_parameter<std::string>("source_frame_id", "camera_link");
  this->get_parameter("source_frame_id", source_frame_id);
  this->declare_parameter<std::string>("odom_topic", "/camera/pose/sample");
  this->get_parameter("odom_topic", odom_topic);
  this->declare_parameter<double>("output_rate", 30.0);
  this->get_parameter("output_rate", output_rate);
  this->declare_parameter<double>("gamma_world", -1.5707963);
  this->get_parameter("gamma_world", gamma_world);
  this->declare_parameter<double>("roll_cam", 0.0);
  this->get_parameter("roll_cam", roll_cam);
  this->declare_parameter<double>("pitch_cam", 0.349066);
  this->get_parameter("pitch_cam", pitch_cam);
  this->declare_parameter<double>("yaw_cam", 0.0);
  this->get_parameter("yaw_cam", yaw_cam);
  this->declare_parameter<double>("linear_accel_cov", 0.01);
  this->get_parameter("linear_accel_cov", linear_accel_cov);
  this->declare_parameter<double>("angular_vel_cov", 0.01);
  this->get_parameter("angular_vel_cov", angular_vel_cov);
  this->declare_parameter<double>("pose_jump_threshold", 0.1);
  this->get_parameter("pose_jump_threshold", pose_jump_threshold);

  auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos,
      std::bind(&VisionToMavros::odometryCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "target_frame_id: %s",
              target_frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "source_frame_id: %s",
              source_frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "odom_topic: %s", odom_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "output_rate: %.1f Hz", output_rate);
  RCLCPP_INFO(this->get_logger(), "gamma_world: %.4f rad (%.1f deg)",
              gamma_world, gamma_world * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "roll_cam: %.4f rad (%.1f deg)", roll_cam,
              roll_cam * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "pitch_cam: %.4f rad (%.1f deg)", pitch_cam,
              pitch_cam * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "yaw_cam: %.4f rad (%.1f deg)", yaw_cam,
              yaw_cam * 180.0 / M_PI);
}

void VisionToMavros::precisionLandParameters(void) {
  this->declare_parameter<bool>("enable_precland", false);
  this->get_parameter("enable_precland", enable_precland);

  RCLCPP_INFO(this->get_logger(), "Precision landing: %s",
              enable_precland ? "enabled" : "disabled");

  if (enable_precland) {
    this->declare_parameter<std::string>("precland_target_frame_id",
                                         "/landing_target");
    this->get_parameter("precland_target_frame_id", precland_target_frame_id);
    this->declare_parameter<std::string>("precland_camera_frame_id",
                                         "/camera_fisheye2_optical_frame");
    this->get_parameter("precland_camera_frame_id", precland_camera_frame_id);

    RCLCPP_INFO(this->get_logger(), "precland_target_frame_id: %s",
                precland_target_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "precland_camera_frame_id: %s",
                precland_camera_frame_id.c_str());

    precland_msg_publisher =
        this->create_publisher<mavros_msgs::msg::LandingTarget>("landing_raw",
                                                                10);
  }
}

void VisionToMavros::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(velocity_mutex);

  velocity_cam.setX(msg->twist.twist.linear.x);
  velocity_cam.setY(msg->twist.twist.linear.y);
  velocity_cam.setZ(msg->twist.twist.linear.z);
  velocity_covariance_cam = msg->twist.covariance;
  pose_covariance_cam = msg->pose.covariance;
  velocity_received = true;
}

static bool isCovarianceValid(const std::array<double, 36> &cov) {
  return (cov[0] > 0.0 || cov[7] > 0.0 || cov[14] > 0.0);
}

std::array<double, 36>
VisionToMavros::calculatePoseCovariance(int confidence_level) {
  confidence_level = std::max(0, std::min(3, confidence_level));
  double cov_pose = linear_accel_cov * std::pow(10.0, 3 - confidence_level);
  double cov_orientation =
      angular_vel_cov * std::pow(10.0, 1 - confidence_level);

  std::array<double, 36> covariance = {};
  covariance[0] = cov_pose;
  covariance[7] = cov_pose;
  covariance[14] = cov_pose;
  covariance[21] = cov_orientation;
  covariance[28] = cov_orientation;
  covariance[35] = cov_orientation;
  return covariance;
}

std::array<double, 36>
VisionToMavros::calculateTwistCovariance(int confidence_level) {
  confidence_level = std::max(0, std::min(3, confidence_level));
  double cov_vel = linear_accel_cov * std::pow(10.0, 3 - confidence_level);

  std::array<double, 36> covariance = {};
  covariance[0] = cov_vel;
  covariance[7] = cov_vel;
  covariance[14] = cov_vel;
  covariance[21] = 1e6;
  covariance[28] = 1e6;
  covariance[35] = 1e6;
  return covariance;
}

void VisionToMavros::detectPoseJump(const tf2::Vector3 &current_pos) {
  if (!first_pose_received) {
    prev_position_body = current_pos;
    first_pose_received = true;
    return;
  }

  tf2::Vector3 delta = current_pos - prev_position_body;
  double displacement = delta.length();

  if (displacement > pose_jump_threshold) {
    RCLCPP_WARN(this->get_logger(), "Pose jump detected! Displacement: %.3f m",
                displacement);
    reset_counter++;
    if (reset_counter == 0)
      reset_counter = 1;
    RCLCPP_INFO(this->get_logger(), "Reset counter: %d", reset_counter);
  }

  prev_position_body = current_pos;
}

void VisionToMavros::transformReady(
    const std::shared_future<geometry_msgs::msg::TransformStamped> &transform) {
  RCLCPP_INFO(this->get_logger(), "Transform result: %f %f %f",
              transform.get().transform.translation.x,
              transform.get().transform.translation.y,
              transform.get().transform.translation.z);
}

bool VisionToMavros::waitForFirstTransform(double timeout = 12.0) {
  bool received = false;
  std::string error_msg;
  auto start_time = this->now();

  RCLCPP_INFO(this->get_logger(), "Waiting for transform '%s' -> '%s'...",
              target_frame_id.c_str(), source_frame_id.c_str());

  rclcpp::Rate rate(3.0);
  while (rclcpp::ok() &&
         (this->now() - start_time < rclcpp::Duration::from_seconds(timeout))) {
    if (buffer->canTransform(target_frame_id, source_frame_id,
                             this->get_clock()->now(),
                             rclcpp::Duration::from_seconds(3.0), &error_msg)) {
      received = true;
      break;
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Waiting... (%s)", error_msg.c_str());
    }
    rate.sleep();
  }

  if (!received) {
    RCLCPP_ERROR(this->get_logger(), "Timeout after %.1f seconds", timeout);
  } else {
    RCLCPP_INFO(this->get_logger(), "Transform available");
  }

  return received;
}

void VisionToMavros::run(void) {
  RCLCPP_INFO(this->get_logger(), "Starting Vision To MAVROS");

  if (!this->waitForFirstTransform(12.0))
    return;

  RCLCPP_INFO(this->get_logger(), "Starting publishing loop at %.1f Hz",
              output_rate);
  this->last_tf_time = this->get_clock()->now();

  auto timer = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / this->output_rate)),
      std::bind(&VisionToMavros::publishVisionPositionEstimate, this));

  rclcpp::spin(this->shared_from_this());
}

void VisionToMavros::publishVisionPositionEstimate() {
  try {
    transform_stamped = buffer->lookupTransform(
        target_frame_id, source_frame_id, tf2::TimePointZero);

    if (last_tf_time < transform_stamped.header.stamp) {
      last_tf_time = transform_stamped.header.stamp;

      tf2::fromMsg(transform_stamped.transform.translation, position_orig);
      position_body.setX(cos(gamma_world) * position_orig.getX() +
                         sin(gamma_world) * position_orig.getY());
      position_body.setY(-sin(gamma_world) * position_orig.getX() +
                         cos(gamma_world) * position_orig.getY());
      position_body.setZ(position_orig.getZ());

      tf2::fromMsg(transform_stamped.transform.rotation, quat_cam);
      quat_cam_to_body.setRPY(roll_cam, pitch_cam, yaw_cam);
      quat_rot_z.setRPY(0, 0, -gamma_world);
      quat_body = quat_rot_z * quat_cam * quat_cam_to_body;
      quat_body.normalize();

      if (std::isnan(position_body.getX()) ||
          std::isnan(position_body.getY()) ||
          std::isnan(position_body.getZ()) || std::isnan(quat_body.getX()) ||
          std::isnan(quat_body.getY()) || std::isnan(quat_body.getZ()) ||
          std::isnan(quat_body.getW())) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "NaN in pose — T265 tracking lost. Dropping frame.");
        return;
      }

      detectPoseJump(position_body);

      msg_body_pose.header.stamp = transform_stamped.header.stamp;
      msg_body_pose.header.frame_id = transform_stamped.header.frame_id;
      msg_body_pose.pose.position.x = position_body.getX();
      msg_body_pose.pose.position.y = position_body.getY();
      msg_body_pose.pose.position.z = position_body.getZ();
      msg_body_pose.pose.orientation.x = quat_body.getX();
      msg_body_pose.pose.orientation.y = quat_body.getY();
      msg_body_pose.pose.orientation.z = quat_body.getZ();
      msg_body_pose.pose.orientation.w = quat_body.getW();
      camera_pose_publisher->publish(msg_body_pose);

      geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
      pose_cov_msg.header = msg_body_pose.header;
      pose_cov_msg.pose.pose = msg_body_pose.pose;
      {
        std::lock_guard<std::mutex> lock(velocity_mutex);
        if (isCovarianceValid(pose_covariance_cam)) {
          pose_cov_msg.pose.covariance = pose_covariance_cam;
        } else {
          RCLCPP_WARN_ONCE(this->get_logger(),
                           "T265 pose covariance is zero — using fallback.");
          pose_cov_msg.pose.covariance = calculatePoseCovariance(3);
        }
      }
      vision_pose_cov_publisher->publish(pose_cov_msg);

      if (velocity_received) {
        std::lock_guard<std::mutex> lock(velocity_mutex);

        tf2::Vector3 velocity_odom = tf2::quatRotate(quat_cam, velocity_cam);

        tf2::Vector3 velocity_world;
        velocity_world.setX(cos(gamma_world) * velocity_odom.getX() +
                            sin(gamma_world) * velocity_odom.getY());
        velocity_world.setY(-sin(gamma_world) * velocity_odom.getX() +
                            cos(gamma_world) * velocity_odom.getY());
        velocity_world.setZ(velocity_odom.getZ());

        if (std::isnan(velocity_world.getX()) ||
            std::isnan(velocity_world.getY()) ||
            std::isnan(velocity_world.getZ())) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "NaN in velocity — dropping frame.");
        } else {
          geometry_msgs::msg::TwistWithCovarianceStamped speed_msg;
          speed_msg.header = msg_body_pose.header;
          speed_msg.twist.twist.linear.x = velocity_world.getX();
          speed_msg.twist.twist.linear.y = velocity_world.getY();
          speed_msg.twist.twist.linear.z = velocity_world.getZ();
          speed_msg.twist.twist.angular.x = 0.0;
          speed_msg.twist.twist.angular.y = 0.0;
          speed_msg.twist.twist.angular.z = 0.0;

          if (velocity_covariance_cam[0] > 0) {
            speed_msg.twist.covariance = velocity_covariance_cam;
          } else {
            speed_msg.twist.covariance = calculateTwistCovariance(3);
          }
          vision_speed_publisher->publish(speed_msg);
        }
      }

      body_path.header.stamp = msg_body_pose.header.stamp;
      body_path.header.frame_id = msg_body_pose.header.frame_id;
      body_path.poses.push_back(msg_body_pose);
      body_path_publisher->publish(body_path);

      tf2::Matrix3x3 m(quat_body);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Pos:[%.2f,%.2f,%.2f] RPY:[%.1f,%.1f,%.1f] Reset:%d",
                           position_body.getX(), position_body.getY(),
                           position_body.getZ(), roll * 180.0 / M_PI,
                           pitch * 180.0 / M_PI, yaw * 180.0 / M_PI,
                           reset_counter);
    }
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s",
                         ex.what());
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionToMavros>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
