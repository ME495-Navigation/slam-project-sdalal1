/// \file turtle_control.cpp
/// \brief Controlling the turtle parameters
///
/// PARAMETERS:
///     wheel_radius (double): radius of the wheels
///     track_width (double): width between the left and right wheels
///     motor_cmd_max (double): maximum possible wheel velocity
///     motor_cmd_per_rad_sec (double): motor cmd vel per radian sec
///     encoder_ticks_per_rad (double): motor encoder ticks per radians
///     collision_radius (double): the collision radius of the robot
/// PUBLISHES:
///     wheel_cmd (nuturtlebot_msgs::msg::WheelCommands):  command for each wheel to move at a certain speed
///     joint_states (sensor_msgs::msg::JointState):  contains information about the current state of all the joints in the robot
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::msg::Twist): The twist of the moving robot
///     sensor_data (nuturtlebot_msgs::msg::SensorData): The sensor data from the turtlebot
/// SERVICES:
///     None
/// BROADCASTS:
///     None

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"

using namespace std::chrono_literals;
using turtlelib::almost_equal;

/// \brief Controller to set paramters for the robot
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("motor_cmd_max", 265);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 1.0 / 0.00153398078);
    declare_parameter("collision_radius", 0.11);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();
    velo_subscription = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::velo_callback, this, std::placeholders::_1));
    velo_publish = create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10);
    sensor_subscription = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_callback, this, std::placeholders::_1));
    joint_publish = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);

    if (wheel_radius_ == 0.0 || track_width_ == 0.0 || motor_cmd_max_ == 0.0 ||
      motor_cmd_per_rad_sec_ == 0.0 || encoder_ticks_per_rad_ == 0.0 || collision_radius_ == 0.0)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "a paramter not defined not defined");
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_radius: " << wheel_radius_);
      RCLCPP_ERROR_STREAM(this->get_logger(), "track_width: " << track_width_);
      RCLCPP_ERROR_STREAM(this->get_logger(), "motor_cmd_max: " << motor_cmd_max_);
      RCLCPP_ERROR_STREAM(this->get_logger(), "encoder_ticks_per_rad: " << encoder_ticks_per_rad_);
      RCLCPP_ERROR_STREAM(this->get_logger(), "collision_radius: " << collision_radius_);
      throw std::runtime_error("runtime_error_exception");
    }
  }

private:
  double wheel_radius_ = 0.0;
  double track_width_ = 0.0;
  int motor_cmd_max_ = 0.0;
  double motor_cmd_per_rad_sec_ = 0.0;
  double encoder_ticks_per_rad_ = 0.0;
  double collision_radius_ = 0.0;
  bool checker = false;
  nuturtlebot_msgs::msg::SensorData old_sensor_data;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velo_subscription;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr velo_publish;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_subscription;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publish;

  /// \brief  Callback for the cmd_vel topic subscription
  /// \param msg  The incoming message that provides the twist of the body frame
  void velo_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto a = msg->angular.z;
    auto lx = msg->linear.x;
    auto ly = msg->linear.y;
    turtlelib::DiffDrive diff(track_width_, wheel_radius_);
    turtlelib::Twist2D twist{a, lx, ly};
    const auto wheel_angle = diff.compute_ik(twist);
    auto pub_wheel = nuturtlebot_msgs::msg::WheelCommands();
    pub_wheel.left_velocity = static_cast<int32_t>(wheel_angle.left / motor_cmd_per_rad_sec_);
    pub_wheel.right_velocity = static_cast<int32_t>(wheel_angle.right / motor_cmd_per_rad_sec_);

    if (pub_wheel.left_velocity > motor_cmd_max_) {
      pub_wheel.left_velocity = motor_cmd_max_;
    }
    if (pub_wheel.right_velocity > motor_cmd_max_) {
      pub_wheel.right_velocity = motor_cmd_max_;
    }
    if (pub_wheel.left_velocity < -motor_cmd_max_) {
      pub_wheel.left_velocity = -motor_cmd_max_;
    }
    if (pub_wheel.right_velocity < -motor_cmd_max_) {
      pub_wheel.right_velocity = -motor_cmd_max_;
    }
    velo_publish->publish(pub_wheel);
  }

  /// \brief  callback function to handle sensor data
  /// \param msg Sensor data from sensors on NuturtleBot and simulaiton values
  void sensor_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
  {

    sensor_msgs::msg::JointState js_pub;
    auto left_en = msg->left_encoder;
    auto right_en = msg->right_encoder;
    auto current_time = msg->stamp;
    js_pub.header.stamp = current_time;
    js_pub.name = {"wheel_left_joint", "wheel_right_joint"};
    auto left_joint = static_cast<double>(left_en) / encoder_ticks_per_rad_;
    auto right_joint = static_cast<double>(right_en) / encoder_ticks_per_rad_;

    js_pub.position = std::vector<double>(2);
    js_pub.position.at(0) = left_joint;
    js_pub.position.at(1) = right_joint;

    if (checker) {
      auto old_left_en = old_sensor_data.left_encoder;
      auto old_right_en = old_sensor_data.right_encoder;
      auto last_timestamp = old_sensor_data.stamp.nanosec;

      auto dt = (current_time.nanosec - last_timestamp) * (1e-9) +
        (current_time.sec - old_sensor_data.stamp.sec);
      js_pub.velocity = std::vector<double>(2);
      js_pub.velocity.at(0) = (left_en - old_left_en) / (dt * encoder_ticks_per_rad_);
      js_pub.velocity.at(1) = (right_en - old_right_en) / (dt * encoder_ticks_per_rad_);

      joint_publish->publish(js_pub);
      old_sensor_data = *msg;
    } else {
      // RCLCPP_INFO_STREAM(this->get_logger(), "first run");
      old_sensor_data = *msg;
      checker = true;
    }

  }
};

/// \brief The main function to spin the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;

}
