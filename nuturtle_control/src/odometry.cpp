/// \file odometry.cpp
/// \brief Contains the implementation of the Odometry class
///
/// PARAMETERS:
///     wheel_radius (double): radius of the wheels
///     track_width (double): width between the left and right wheels
///     body_id (string): body frame link name
///     odom_id (string): odom frame link name (defaults: odom)
///     wheel_left (string):  left wheel link name
///     wheel_right (string): right wheel link name
/// PUBLISHES:
///      /odom (nav_msgs::msg::Odometry): contains the odometry message
/// SUBSCRIBES:
///      /joint_states (sensor_msgs::msg::JointState): joint states from sim or real robot
/// SERVICES:
///      initial_pose (nuturtle_control::srv::InitialPose): Sets the initial pose of blue robot
/// BROADCASTS:
///      odom_id -> base_id

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

/// \brief  Odometry node implements the main turtlebot functionality using DiffDrive class.
class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("body_id", " ");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", " ");
    declare_parameter("wheel_right", " ");

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();

    if (body_id == " " || wheel_left == " " || wheel_right == " " || wheel_radius_ == 0.0 ||
      track_width_ == 0.0)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "a paramter not defined not defined");
      RCLCPP_ERROR_STREAM(this->get_logger(), "body_id: " << body_id);
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_left: " << wheel_left);
      RCLCPP_ERROR_STREAM(this->get_logger(), "Wheel_right: " << wheel_right);
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_radius: " << wheel_radius_);
      RCLCPP_ERROR_STREAM(this->get_logger(), "track_width: " << track_width_);
      throw std::runtime_error("runtime_error_exception");
    }
    joint_subscription = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::joint_callback, this, std::placeholders::_1));
    srv_initial_pose =
      create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(
        &Odometry::initial_pose_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    blue_path_pub = create_publisher<nav_msgs::msg::Path>("blue/path", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = create_wall_timer(
      1000ms / 200, std::bind(&Odometry::timer_callback, this));

    tr = {};
    diff = std::make_unique<turtlelib::DiffDrive>(tr, track_width_, wheel_radius_);

  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::string body_id = " ";
  std::string odom_id = " ";
  std::string wheel_left = " ";
  std::string wheel_right = " ";
  double wheel_radius_ = 0.0;
  double track_width_ = 0.0;
  bool checker = false;
  sensor_msgs::msg::JointState old_js, js_pos;
  turtlelib::Transform2D tr;
  std::unique_ptr<turtlelib::DiffDrive> diff;
  turtlelib::Transform2D transformation;
  tf2::Quaternion q;
  nav_msgs::msg::Odometry odom_pub = nav_msgs::msg::Odometry();
  geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();
  nav_msgs::msg::Path blue_path = nav_msgs::msg::Path();
  geometry_msgs::msg::PoseStamped ps = geometry_msgs::msg::PoseStamped();

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blue_path_pub;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr srv_initial_pose;

  /// \brief timer callbacck publishing transforms and odometry
  void timer_callback()
  {

    tf_broadcaster_->sendTransform(t);
    odom_publisher->publish(odom_pub);

    blue_path.header.stamp = this->get_clock()->now();
  }

  /// \brief  callback function for the joint state subscription
  /// \param msg The Joint State message from the topic /joint_states
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "in joint callback pose");
    js_pos.position = msg->position;

    if (checker) {
      // RCLCPP_INFO_STREAM(this->get_logger(), "cchecker_triggered");
      auto old_js_left = old_js.position.at(0);
      auto old_js_right = old_js.position.at(1);
      // RCLCPP_INFO_STREAM(this->get_logger(), "old_left"<<old_js_left);
      // RCLCPP_INFO_STREAM(this->get_logger(), "old_right"<<old_js_right);

      auto pos_left = js_pos.position.at(0);
      auto pos_right = js_pos.position.at(1);
      // RCLCPP_INFO_STREAM(this->get_logger(), "new_left"<<pos_left);
      // RCLCPP_INFO_STREAM(this->get_logger(), "new_right"<<pos_right);

      auto delta_js_left = pos_left - old_js_left;
      auto delta_js_right = pos_right - old_js_right;

      diff->compute_fk(delta_js_left, delta_js_right);
      // RCLCPP_INFO_STREAM(this->get_logger(), "left delta"<<delta_js_left);
      // RCLCPP_INFO_STREAM(this->get_logger(), "right delta"<<delta_js_right);

      // RCLCPP_INFO_STREAM(this->get_logger(), "transformation" << transformation);
      transformation = diff->get_transformation();

      odom_pub.header.stamp = this->get_clock()->now();
      odom_pub.header.frame_id = odom_id;
      odom_pub.child_frame_id = body_id;

      odom_pub.pose.pose.position.x = transformation.translation().x;
      odom_pub.pose.pose.position.y = transformation.translation().y;

      q.setRPY(0, 0, transformation.rotation());

      odom_pub.pose.pose.orientation.x = q.x();
      odom_pub.pose.pose.orientation.y = q.y();
      odom_pub.pose.pose.orientation.z = q.z();
      odom_pub.pose.pose.orientation.w = q.w();

      odom_pub.twist.twist.linear.x = diff->get_twist().x;
      odom_pub.twist.twist.linear.y = diff->get_twist().y;
      odom_pub.twist.twist.linear.z = 0.0;
      odom_pub.twist.twist.angular.x = 0.0;
      odom_pub.twist.twist.angular.y = 0.0;
      odom_pub.twist.twist.angular.z = diff->get_twist().omega;


      t.header.frame_id = odom_id;
      t.child_frame_id = body_id;
      t.header.stamp = this->get_clock()->now();
      t.transform.translation.x = transformation.translation().x;
      t.transform.translation.y = transformation.translation().y;
      q.setRPY(0, 0, transformation.rotation());
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      blue_path.header.frame_id = odom_id;
      ps.header.frame_id = odom_id;
      ps.pose.position.x = transformation.translation().x;
      ps.pose.position.y = transformation.translation().y;
      tf2::Quaternion q_red;
      q_red.setRPY(0, 0, transformation.rotation());
      ps.pose.orientation.x = q_red.x();
      ps.pose.orientation.y = q_red.y();
      ps.pose.orientation.z = q_red.z();
      ps.pose.orientation.w = q_red.w();

      blue_path.poses.push_back(ps);
      if(blue_path.poses.size() > 7000){
        blue_path.poses.erase(blue_path.poses.begin());
      }
      blue_path_pub->publish(blue_path);


      old_js = *msg;
    } else {
      checker = true;
      old_js = *msg;
    }
  }

  /// \brief Call back for initial pose service
  /// \param req Inital pose request from srv
  void initial_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    double x_pos = request->x;
    double y_pos = request->y;
    auto theta_pos = request->theta;

    turtlelib::Transform2D trans{turtlelib::Vector2D{x_pos, y_pos}, theta_pos};
    turtlelib::DiffDrive new_diff{trans, track_width_, wheel_radius_};
    diff->change_transform(trans);
  }
};

/// \brief The main function to spin the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;

}
