/// \file nusim.cpp
/// \brief Simulating the control of turtlebot
///
/// PARAMETERS:
///     rate (double): rate of the timer (in Hz)
///     x0 (double): start location of the red bot (m)
///     y0 (double): start location of the red bot (m)
///     theta0 (double): start orientation of the red bot (rad)
///     obstacles/x (double[]): Obstacles x position wrt nusim/world (m)
///     obstacles/y (double[]): Obstacles y position wrt nusim/world (m)
///     obstacles/r (double): radius of the obstacles (cylinder) (m)
///     arena_x_length: length of arena (m)
///     arena_y_length: width of arena (m)
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::Uint64): Inerations of the simulation
///     ~/obstacles (visualization_msgs::msg::MarkerArray): cylinder objects in RVIZ
///     ~/walls (visualization_msgs::msg::MarkerArray): arena in RVIZ
/// SERVICES:
///     ~/reset (std_srvs::srv::Empty): resets simulation to the intial values
///     ~/teleport (nusim::srv::Teleport): teleports the robot in simulation to the given position and orientation
/// BROADCASTS:
///    nusim/world -> red/base_footprint

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "std_msgs/msg/u_int64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

/// \brief Node to start simulation in RVIZ
class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim")
  {
    declare_parameter("rate", 200.0);
    declare_parameter("x0", x0);
    declare_parameter("y0", y0);
    declare_parameter("theta0", theta0);
    declare_parameter("obstacle/x", obs_x);
    declare_parameter("obstacle/y", obs_y);
    declare_parameter("obstacle/r", obs_r);
    declare_parameter("arena_x_length", 5.0);
    declare_parameter("arena_y_length", 10.0);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);

    rate = get_parameter("rate").as_double();
    x0 = get_parameter("x0").as_double();
    y0 = get_parameter("y0").as_double();
    theta0 = get_parameter("theta0").as_double();
    obs_x = get_parameter("obstacle/x").as_double_array();
    obs_y = get_parameter("obstacle/y").as_double_array();
    obs_r = get_parameter("obstacle/r").as_double();
    arena_x_length = get_parameter("arena_x_length").as_double();
    arena_y_length = get_parameter("arena_y_length").as_double();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    timer_ = create_wall_timer(
      1000ms / rate, std::bind(&Nusim::timer_callback, this));
    srv_reset =
      create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_cb, this, std::placeholders::_1, std::placeholders::_2));
    srv_teleport =
      create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_cb, this, std::placeholders::_1, std::placeholders::_2));
    publisher_timestep =
      create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    publisher_walls =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos_profile);
    publisher_obs = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      qos_profile);

    red_wheel_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::red_wheel_callback, this, std::placeholders::_1));
    
    sensor_pub = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    ground_frame_loc(x0, y0, theta0);
    publish_walls();
    publish_obs();
  }

private:
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr srv_teleport;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_timestep;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_walls;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_obs;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // C.7 sub
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr red_wheel_sub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  visualization_msgs::msg::MarkerArray arr = visualization_msgs::msg::MarkerArray();
  visualization_msgs::msg::MarkerArray ob = visualization_msgs::msg::MarkerArray();
  visualization_msgs::msg::Marker w1 = visualization_msgs::msg::Marker();
  visualization_msgs::msg::Marker w2 = visualization_msgs::msg::Marker();
  visualization_msgs::msg::Marker w3 = visualization_msgs::msg::Marker();
  visualization_msgs::msg::Marker w4 = visualization_msgs::msg::Marker();

  geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();

  double rate = 0.0;
  double x0 = 0.0;
  double y0 = 0.0;
  double theta0 = 0.0;
  double arena_x_length = 0.0;
  double arena_y_length = 0.0;
  double obs_r = 0.0;
  std::vector<double> obs_x = {2.0, 3.2, 4.2};
  std::vector<double> obs_y = {3.0, 5.2, 2.1};

  double left_wheel, right_wheel;
  nuturtlebot_msgs::msg::SensorData red_sensor;
  double motor_cmd_per_rad_sec_ = 0.0;
  double wheel_radius_ = 0.033;
  double track_width_ = 0.16;

  turtlelib::Transform2D tr{};
  turtlelib::DiffDrive diff{tr, track_width_, wheel_radius_};
  bool checker = false;

  nuturtlebot_msgs::msg::WheelCommands old_wheels, new_wheels;

  /// \brief Creates the obstacle an publishes it
  void publish_obs()
  {
    int i = 0;
    if (obs_x.size() == obs_y.size()) {
      for (i = 0; i < int(obs_x.size()); i++) {
        visualization_msgs::msg::Marker cyl = visualization_msgs::msg::Marker();
        cyl.header.stamp = this->get_clock()->now();
        cyl.header.frame_id = "nusim/world";
        cyl.id = i + 1;
        cyl.type = visualization_msgs::msg::Marker::CYLINDER;
        cyl.action = visualization_msgs::msg::Marker::ADD;
        cyl.scale.x = obs_r * 2;
        cyl.scale.y = obs_r * 2;
        cyl.scale.z = 0.25;
        cyl.pose.position.x = obs_x[i];
        cyl.pose.position.y = obs_y[i];
        cyl.pose.position.z = 0.25 / 2;
        cyl.color.r = 1.0;
        cyl.color.g = 0.0;
        cyl.color.b = 0.0;
        cyl.color.a = 1.0;

        ob.markers.push_back(cyl);
        publisher_obs->publish(ob);
      }
    } else {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "The size of x and y are not equal," << "size of x" << obs_x.size() << ",size of y" <<
          obs_y.size());
    }
  }

  /// \brief Creates the walls and publish it
  void publish_walls()
  {
    w1.header.stamp = this->get_clock()->now();
    w1.header.frame_id = "nusim/world";
    w1.id = 1;
    w1.type = visualization_msgs::msg::Marker::CUBE;
    w1.action = visualization_msgs::msg::Marker::ADD;
    w1.scale.x = 0.1;
    w1.scale.y = arena_y_length;
    w1.scale.z = 0.25;
    w1.pose.position.x = arena_x_length / 2;
    w1.pose.position.y = 0.0;
    w1.pose.position.z = 0.0;
    w1.color.r = 1.0;
    w1.color.g = 0.0;
    w1.color.b = 0.0;
    w1.color.a = 1.0;

    w2.header.stamp = this->get_clock()->now();
    w2.header.frame_id = "nusim/world";
    w2.id = 2;
    w2.type = visualization_msgs::msg::Marker::CUBE;
    w2.action = visualization_msgs::msg::Marker::ADD;
    w2.scale.x = 0.1;
    w2.scale.y = arena_y_length;
    w2.scale.z = 0.25;
    w2.pose.position.x = -arena_x_length / 2;
    w2.pose.position.y = 0.0;
    w2.pose.position.z = 0.0;
    w2.color.r = 1.0;
    w2.color.g = 0.0;
    w2.color.b = 0.0;
    w2.color.a = 1.0;

    w3.header.stamp = this->get_clock()->now();
    w3.header.frame_id = "nusim/world";
    w3.id = 3;
    w3.type = visualization_msgs::msg::Marker::CUBE;
    w3.action = visualization_msgs::msg::Marker::ADD;
    w3.scale.x = arena_x_length;
    w3.scale.y = 0.1;
    w3.scale.z = 0.25;
    w3.pose.position.x = 0.0;
    w3.pose.position.y = arena_y_length / 2;
    w3.pose.position.z = 0.0;
    w3.color.r = 1.0;
    w3.color.g = 0.0;
    w3.color.b = 0.0;
    w3.color.a = 1.0;

    w4.header.stamp = this->get_clock()->now();
    w4.header.frame_id = "nusim/world";
    w4.id = 4;
    w4.type = visualization_msgs::msg::Marker::CUBE;
    w4.action = visualization_msgs::msg::Marker::ADD;
    w4.scale.x = arena_x_length;
    w4.scale.y = 0.1;
    w4.scale.z = 0.25;
    w4.pose.position.x = 0.0;
    w4.pose.position.y = -arena_y_length / 2;
    w4.pose.position.z = 0.0;
    w4.color.r = 1.0;
    w4.color.g = 0.0;
    w4.color.b = 0.0;
    w4.color.a = 1.0;

    arr.markers.push_back(w1);
    arr.markers.push_back(w2);
    arr.markers.push_back(w3);
    arr.markers.push_back(w4);

    publisher_walls->publish(arr);

  }

  /// \brief creates a transform between nusim/world and red/base_foorprint
  /// \param x The x translation of the robot
  /// \param x The y translation of the robot
  /// \param theta The rotation of the robot
  void ground_frame_loc(double x, double y, double theta)
  {
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
  }

  /// \brief Creates a timer to publish frames and timestep
  void timer_callback()
  {
    auto time_msg = std_msgs::msg::UInt64();
    count_++;
    time_msg.data = count_;
    publisher_timestep->publish(time_msg);

    red_sensor.stamp = this->get_clock()->now();


    t.header.stamp = this->get_clock()->now();

    tf_broadcaster_->sendTransform(t);

    sensor_pub -> publish(red_sensor);
  }

  /// \brief The service to teleport the bot in rviz
  /// \param request The request to get the x, y and theta values
  void teleport_cb(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    const std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    ground_frame_loc(request->x, request->y, request->theta);
  }

  /// \brief The reset service to reset the simulation to its initial state
  void reset_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    count_ = 0;
    ground_frame_loc(x0, y0, theta0);
    RCLCPP_INFO_STREAM(get_logger(), "resetting all variables" << x0 << y0 << theta0);
  }

  void red_wheel_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg ){
    // left_wheel = msg->left_velocity;
    // right_wheel = msg->right_velocity;

    // if (checker){
      new_wheels.left_velocity = msg->left_velocity;
      new_wheels.right_velocity = msg->right_velocity;

      // red_sensor.left_encoder = new_wheels.left_velocity*motor_cmd_per_rad_sec_/rate;
      // red_sensor.right_encoder = new_wheels.right_velocity*motor_cmd_per_rad_sec_/rate;

      red_sensor.left_encoder = new_wheels.left_velocity;
      red_sensor.right_encoder = new_wheels.right_velocity;

      diff.compute_fk(new_wheels.left_velocity*motor_cmd_per_rad_sec_/rate,new_wheels.right_velocity*motor_cmd_per_rad_sec_/rate);
      auto trans_red = diff.get_transformation();
      ground_frame_loc(trans_red.translation().x, trans_red.translation().y, trans_red.rotation());
  }

  size_t count_{0};
};

/// \brief The main function to spin the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;

}
