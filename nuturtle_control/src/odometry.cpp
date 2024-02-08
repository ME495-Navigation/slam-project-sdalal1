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

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("body_id", "base_footprint");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "left_wheel");
    declare_parameter("wheel_right", "right_right");

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();

    RCLCPP_INFO_STREAM(this->get_logger(), "body_id"<<body_id);
    joint_subscription = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&Odometry::joint_callback, this, std::placeholders::_1));
    srv_initial_pose =
      create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(&Odometry::initial_pose_callback, this, std::placeholders::_1, std::placeholders::_2));
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = create_wall_timer(
      1000ms/200, std::bind(&Odometry::timer_callback, this));
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::string body_id;
  std::string odom_id;
  std::string wheel_left;
  std::string wheel_right;
  double wheel_radius_ = 0.033;
  double track_width_ = 0.16;
  bool checker = false;
  sensor_msgs::msg::JointState old_js, js_pos;
  turtlelib::Transform2D tr{};
  turtlelib::DiffDrive diff{tr, track_width_, wheel_radius_};
  turtlelib::Transform2D transformation;
  tf2::Quaternion q;
  nav_msgs::msg::Odometry odom_pub = nav_msgs::msg::Odometry();
  geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr srv_initial_pose;

  void timer_callback()
  {
    
    tf_broadcaster_ -> sendTransform(t);
    odom_publisher -> publish(odom_pub);
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
    // RCLCPP_INFO_STREAM(this->get_logger(), "in joint callback pose");
    js_pos.position = msg->position;
    // auto js_velo = msg->velocity;
    if(checker){
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

      diff.compute_fk(delta_js_left, delta_js_right);
      // RCLCPP_INFO_STREAM(this->get_logger(), "left delta"<<delta_js_left);
      // RCLCPP_INFO_STREAM(this->get_logger(), "right delta"<<delta_js_right);

      // RCLCPP_INFO_STREAM(this->get_logger(), "transformation" << transformation);
      transformation  = diff.get_transformation();

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

      odom_pub.twist.twist.linear.x = diff.get_twist().x;
      odom_pub.twist.twist.linear.y = diff.get_twist().y;
      odom_pub.twist.twist.linear.z = 0.0;
      odom_pub.twist.twist.angular.x = 0.0;
      odom_pub.twist.twist.angular.y = 0.0;
      odom_pub.twist.twist.angular.z = diff.get_twist().omega;
      

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
      

      old_js = *msg;
    }
    else{
      checker = true;
      RCLCPP_INFO_STREAM(this->get_logger(), "checker set to true");

      old_js = *msg;
    }
    };
  void initial_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Response>){
      // RCLCPP_INFO_STREAM(this->get_logger(), "in inital _callback pose");
      double x_pos = request->x;
      double y_pos = request->y;
      auto theta_pos = request->theta;
      // checker = false;

      turtlelib::Transform2D trans{turtlelib::Vector2D{x_pos,y_pos}, theta_pos};
      turtlelib::DiffDrive new_diff{trans,track_width_, wheel_radius_};
      diff = new_diff;
      // checker=true;
  };
};




/// \brief The main function to spin the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;

}