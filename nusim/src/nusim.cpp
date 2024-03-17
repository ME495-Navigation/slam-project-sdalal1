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
///     arena_x_length (vector): length of arena (m)
///     arena_y_length (vector): width of arena (m)
///     motor_cmd_per_rad_sec (double): motor command per radian per second
///     wheel_radius (double): radius of the wheel (m)
///     track_width (double): width of the track (m)
///     collision_radius (double): radius of the collision (m)
///     input_noise (double): noise in the wheel commands
///     slip_fraction (double): fraction of slip
///     basic_sensor_variance (double): basic sensor noise variance
///     max_range (double): maximum range of the sensor
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::Uint64): Inerations of the simulation
///     ~/obstacles (visualization_msgs::msg::MarkerArray): cylinder objects in RVIZ
///     ~/walls (visualization_msgs::msg::MarkerArray): arena in RVIZ
///     red/sensor_data (nusim_ros::msg::SensorData): sensor data of the red bot
///     red/path (nav_msgs::msg::Path): path of the red bot
///     scan (sensor_msgs::msg::LaserScan): laser scan data
/// SUBSCRIBES:
///      /red/cmd_vel (geometry_msgs::msg::Twist): Velocity commands for the red bot
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
#include <random>

#include "std_msgs/msg/u_int64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
    declare_parameter("wheel_radius", 0.0);
    declare_parameter("track_width", 0.0);
    declare_parameter("collision_radius", 0.11);
    declare_parameter("input_noise", 0.0001);
    declare_parameter("slip_fraction", 0.0001);
    declare_parameter("basic_sensor_variance", 0.01);
    declare_parameter("max_range", 1.1);
    declare_parameter("laser_mainimim_range", 0.12);
    declare_parameter("laser_maximum_range", 2 * turtlelib::PI);
    declare_parameter("laser_angle_increment", 0.01745329238474369);
    declare_parameter("laser_noise_variance", 0.001);
    declare_parameter("draw_only", false);

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
    input_noise_ = get_parameter("input_noise").as_double();
    slip_fraction_ = get_parameter("slip_fraction").as_double();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").as_double();
    max_range_ = get_parameter("max_range").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();
    laser_mainimim_range_ = get_parameter("laser_mainimim_range").as_double();
    laser_maximum_range_ = get_parameter("laser_maximum_range").as_double();
    laser_angle_increment_ = get_parameter("laser_angle_increment").as_double();
    laser_noise_variance_ = get_parameter("laser_noise_variance").as_double();
    draw_only = get_parameter("draw_only").as_bool();

    timer_ = create_wall_timer(
      1000ms / rate, std::bind(&Nusim::timer_callback, this));
    fake_timer_ = create_wall_timer(
      200ms, std::bind(&Nusim::fake_timer_callback, this));
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
    // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    publisher_walls =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos_profile);
    rclcpp::QoS qos_profile1(10);
    // qos_profile1.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile1.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    publisher_obs = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      qos_profile);
    publish_fake_obs = create_publisher<visualization_msgs::msg::MarkerArray>(
      "fake_sensor",
      qos_profile1);
    red_path_pub = create_publisher<nav_msgs::msg::Path>("red/path", 10);
    red_wheel_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::red_wheel_callback, this, std::placeholders::_1));
    sensor_pub = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);
    rclcpp::QoS qos_profile2(10);
    qos_profile2.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile2.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    laser_pub = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", qos_profile2);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    ground_frame_loc(x0, y0, theta0);
    tr = {turtlelib::Vector2D{x0, y0}, theta0};
    diff = std::make_unique<turtlelib::DiffDrive>(tr, track_width_, wheel_radius_);
    publish_walls();
    publish_obs();
  }

private:
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr srv_teleport;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_timestep;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_walls;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_obs;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publish_fake_obs;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr fake_timer_;
  // C.7 sub
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr red_wheel_sub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr red_path_pub;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  visualization_msgs::msg::MarkerArray arr = visualization_msgs::msg::MarkerArray();
  visualization_msgs::msg::MarkerArray ob = visualization_msgs::msg::MarkerArray();
  visualization_msgs::msg::Marker w1 = visualization_msgs::msg::Marker();
  visualization_msgs::msg::Marker w2 = visualization_msgs::msg::Marker();
  visualization_msgs::msg::Marker w3 = visualization_msgs::msg::Marker();
  visualization_msgs::msg::Marker w4 = visualization_msgs::msg::Marker();


  geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();
  nav_msgs::msg::Path red_path = nav_msgs::msg::Path();
  geometry_msgs::msg::PoseStamped ps = geometry_msgs::msg::PoseStamped();
  sensor_msgs::msg::LaserScan laser = sensor_msgs::msg::LaserScan();

  visualization_msgs::msg::MarkerArray fake_obs = visualization_msgs::msg::MarkerArray();
  visualization_msgs::msg::Marker fake_cyl = visualization_msgs::msg::Marker();
  visualization_msgs::msg::Marker cyl = visualization_msgs::msg::Marker();


  double rate = 0.0;
  double x0 = 0.0;
  double y0 = 0.0;
  double theta0 = 0.0;
  double arena_x_length = 0.0;
  double arena_y_length = 0.0;
  double obs_r = 0.0;
  std::vector<double> obs_x = {-0.5, 0.8, 0.4};
  std::vector<double> obs_y = {-0.7, -0.8, 0.8};

  double left_wheel = 0.0;
  double right_wheel = 0.0;
  nuturtlebot_msgs::msg::SensorData red_sensor;
  double motor_cmd_per_rad_sec_ = 0.0;
  double wheel_radius_ = 0.033;
  double track_width_ = 0.16;
  double input_noise_ = 0.0;
  double slip_fraction_ = 0.0;
  double basic_sensor_variance_ = 0.0;
  double max_range_ = 0.0;
  double collision_radius_ = 0.0;
  double laser_mainimim_range_ = 0.0;
  double laser_maximum_range_ = 2 * turtlelib::PI;
  double laser_angle_increment_ = 0.01745329238474369;
  double laser_noise_variance_ = 0.0;
  bool draw_only = false;

  double x_robot = 0.0;
  double y_robot = 0.0;
  double theta_robot = 0.0;
  size_t count_{0};

  turtlelib::Transform2D tr;
  std::unique_ptr<turtlelib::DiffDrive> diff;

  nuturtlebot_msgs::msg::WheelCommands old_wheels, new_wheels;

  /// \brief Creates the obstacle an publishes it
  void publish_obs()
  {
    int i = 0;
    if (obs_x.size() == obs_y.size()) {
      for (i = 0; i < int(obs_x.size()); i++) {
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

  /// @brief the laser publisher
  void publish_laser()
  {
    laser.header.frame_id = "red/base_scan";
    laser.angle_min = 0.0;
    laser.angle_max = 2 * turtlelib::PI;
    laser.angle_increment = 0.01745329238474369; // in radians
    laser.time_increment = 0.0; //0.0005592841189354658;
    laser.scan_time = 0.20134228467941284;
    laser.range_min = 0.11999999731779099;
    laser.range_max = 3.5;
    laser.ranges.clear();
    for (double i = laser.angle_min; i < laser.angle_max; i += laser.angle_increment) {
      auto x_lidar_max = laser.range_max * std::cos(i);
      auto y_lidar_max = laser.range_max * std::sin(i);
      turtlelib::Point2D v1{x_lidar_max, y_lidar_max};
      double dist = laser.range_max;
      double mag = laser.range_max;
      auto v2 = diff->get_transformation()(v1);
      auto test = diff->get_transformation()(v1);
      auto slope = (v2.y - diff->get_transformation().translation().y) /
        (v2.x - diff->get_transformation().translation().x);
      auto con = diff->get_transformation().translation().y - slope *
        diff->get_transformation().translation().x;

      // if (v2.y > arena_y_length / 2) {
      //   v2.y = arena_y_length / 2;
      //   v2.x = (arena_y_length / 2 - con) / slope;
      //   auto v3 = diff->get_transformation().inv()(v2);
      //   dist = turtlelib::magnitude(turtlelib::Vector2D{v3.x, v3.y});
      //   dist = std::min(dist, mag);
      // }
      // if (v2.x < -arena_x_length / 2) {
      //   v2.x = -arena_x_length / 2;
      //   v2.y = (slope * -arena_x_length / 2) + con;
      //   auto v3 = diff->get_transformation().inv()(v2);
      //   dist = turtlelib::magnitude(turtlelib::Vector2D{v3.x, v3.y});
      //   dist = std::min(dist, mag);
      // }
      // if (v2.y < -arena_y_length / 2) {
      //   v2.y = -arena_y_length / 2;
      //   v2.x = (-arena_y_length / 2 - con) / slope;
      //   auto v3 = diff->get_transformation().inv()(v2);
      //   dist = turtlelib::magnitude(turtlelib::Vector2D{v3.x, v3.y});
      //   dist = std::min(dist, mag);
      // }
      // if (v2.x > arena_x_length / 2) {
      //   v2.x = arena_x_length / 2;
      //   v2.y = (arena_x_length / 2 * slope) + con;
      //   auto v3 = diff->get_transformation().inv()(v2);
      //   dist = turtlelib::magnitude(turtlelib::Vector2D{v3.x, v3.y});
      //   dist = std::min(dist, mag);
      // }
      for (int j = 0; j < int(obs_x.size()); j++) {
        if (distance(
            obs_x[j], obs_y[j], diff->get_transformation().translation().x,
            diff->get_transformation().translation().y) < mag)
        {
          auto p_dist = abs(slope * obs_x[j] - obs_y[j] + con) / std::sqrt(slope * slope + 1);
          if (p_dist <= obs_r) {
            double d1 = distance(
              obs_x[j], obs_y[j],
              diff->get_transformation().translation().x,
              diff->get_transformation().translation().y);
            double d2 = distance(obs_x[j], obs_y[j], test.x, test.y);

            if (turtlelib::almost_equal(
                std::sqrt(
                  std::pow(
                    d1,
                    2) - std::pow(p_dist, 2)) + std::sqrt(std::pow(d2, 2) - std::pow(p_dist, 2)),
                mag))
            {
              auto d =
                std::sqrt(std::pow(d1, 2) - std::pow(p_dist, 2)) - std::sqrt(
                std::pow(
                  obs_r,
                  2) -
                std::pow(p_dist, 2));
              dist = std::min(dist, d);
            }
          }
        }
      }
      if (dist < laser.range_min || dist >= laser.range_max) {
        dist = 0.0;
      }
      std::normal_distribution<> d1(0.0, laser_noise_variance_);
      auto laser_noise = d1(get_random());

      laser.ranges.push_back(dist + laser_noise);
    }
    if (!draw_only) {
      laser_pub->publish(laser);
    }
  }

  /// \brief distance between two points
  /// \param x1 the x position of the first point
  /// \param y1 the y position of the first point
  /// \param x2 the x position of the second point
  /// \param y2 the y position of the second point
  /// \return the distance between the two points
  double distance(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
  }

  /// @brief random number generator
  /// @return the random number
  std::mt19937 & get_random()
  {
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
  }

  /// @brief fake obstacle publisher
  void fake_timer_callback()
  {
    if (!draw_only) {
      publish_laser();
    }
    fake_cyl.header.frame_id = "red/base_footprint";
    fake_cyl.type = visualization_msgs::msg::Marker::CYLINDER;
    std::normal_distribution<> d1(0.0, basic_sensor_variance_);
    auto noise_fake = d1(get_random());
    for (int i = 0; i < int(obs_x.size()); i++) {
      if (std::sqrt(
          std::pow(
            obs_x[i] - diff->get_transformation().translation().x,
            2) +
          std::pow(
            obs_y[i] - diff->get_transformation().translation().y,
            2)) > (max_range_ - obs_r - collision_radius_))
      {
        fake_cyl.id = i;
        fake_cyl.action = visualization_msgs::msg::Marker::DELETE;
      } else {
        fake_cyl.id = i;
        fake_cyl.scale.x = noise_fake + (obs_r * 2);
        fake_cyl.scale.y = noise_fake + (obs_r * 2);
        fake_cyl.scale.z = 0.25;
        turtlelib::Transform2D Two{turtlelib::Vector2D{obs_x[i], obs_y[i]}, 0.0};
        auto Trc = (diff->get_transformation()).inv() * Two;
        fake_cyl.pose.position.x = Trc.translation().x + noise_fake;
        fake_cyl.pose.position.y = Trc.translation().y + noise_fake;
        fake_cyl.pose.position.z = 0.25 / 2;
        fake_cyl.color.r = 1.0;
        fake_cyl.color.g = 1.0;
        fake_cyl.color.b = 0.0;
        fake_cyl.color.a = 1.0;
        fake_cyl.action = visualization_msgs::msg::Marker::ADD;
      }
      fake_obs.markers.push_back(fake_cyl);
    }
    if (!draw_only) {
      publish_fake_obs->publish(fake_obs);
      fake_obs.markers.clear();
    }
  }

  /// \brief Creates a timer to publish frames and timestep
  void timer_callback()
  {

    // RCLCPP_ERROR_STREAM(this->get_logger(),"x0"<<tr.translation().x);
    auto time_msg = std_msgs::msg::UInt64();
    count_++;
    time_msg.data = count_;
    if (!draw_only) {
      publisher_timestep->publish(time_msg);
    }
    red_sensor.stamp = this->get_clock()->now();
    red_path.header.stamp = this->get_clock()->now();
    cyl.header.stamp = this->get_clock()->now();
    t.header.stamp = this->get_clock()->now();
    ps.header.stamp = this->get_clock()->now();
    laser.header.stamp = this->get_clock()->now();
    w1.header.stamp = this->get_clock()->now();
    w2.header.stamp = this->get_clock()->now();
    w3.header.stamp = this->get_clock()->now();
    w4.header.stamp = this->get_clock()->now();

    if (!draw_only) {
      sensor_pub->publish(red_sensor);
    }
    fake_cyl.header.stamp = this->get_clock()->now();

    tf_broadcaster_->sendTransform(t);
  }

  // void check
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

  /// \brief Callback function for wheel velocity
  /// \param the message to get published wheel commands
  void red_wheel_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    auto left_wheel_velocity = static_cast<double>(msg->left_velocity) * 7.5;
    auto right_wheel_velocity = static_cast<double>(msg->right_velocity) * 7.5;
    // auto left_wheel_velocity = (msg->left_velocity);
    // auto right_wheel_velocity = (msg->right_velocity);
    std::normal_distribution<> d(0.0, input_noise_);
    auto noise = d(get_random());
    std::uniform_real_distribution<> slip_noise{-slip_fraction_, slip_fraction_};
    auto slipping_noise = (1 + slip_noise(get_random()));

    if (left_wheel_velocity != 0.0) {

      left_wheel_velocity += noise;
    }
    if (right_wheel_velocity != 0.0) {
      right_wheel_velocity += noise;
    }
    diff->compute_fk(
      left_wheel_velocity * (motor_cmd_per_rad_sec_ / rate),
      right_wheel_velocity * (motor_cmd_per_rad_sec_ / rate));

    left_wheel += (left_wheel_velocity) * motor_cmd_per_rad_sec_ * slipping_noise * 652.229299363 /
      rate;

    right_wheel += (right_wheel_velocity) * motor_cmd_per_rad_sec_ * slipping_noise *
      652.229299363 / rate;

    red_sensor.left_encoder = left_wheel;
    red_sensor.right_encoder = right_wheel;

    check_collision();
    auto trans_red = diff->get_transformation();
    x_robot = trans_red.translation().x;
    y_robot = trans_red.translation().y;
    theta_robot = trans_red.rotation();
    ground_frame_loc(x_robot, y_robot, theta_robot);

    red_path.header.frame_id = "nusim/world";

    ps.header.frame_id = "nusim/world";
    ps.pose.position.x = x_robot;
    ps.pose.position.y = y_robot;
    tf2::Quaternion q_red;
    q_red.setRPY(0, 0, theta_robot);
    ps.pose.orientation.x = q_red.x();
    ps.pose.orientation.y = q_red.y();
    ps.pose.orientation.z = q_red.z();
    ps.pose.orientation.w = q_red.w();
    red_path.poses.push_back(ps);

    if (red_path.poses.size() >= 7000) {
      red_path.poses.erase(red_path.poses.begin());
    }
    if (!draw_only) {
      red_path_pub->publish(red_path);
    }
  }

  void check_collision()
  {
    for (int i = 0; i < int(obs_x.size()); i++) {
      auto distance = std::sqrt(
        std::pow(
          obs_x.at(
            i) - diff->get_transformation().translation().x,
          2) + std::pow(obs_y.at(i) - diff->get_transformation().translation().y, 2));
      if (distance < (obs_r + collision_radius_)) {
        auto u_x = (diff->get_transformation().translation().x - obs_x.at(i)) / distance;
        auto u_y = (diff->get_transformation().translation().y - obs_y.at(i)) / distance;
        auto x_new = x_robot + (obs_r + collision_radius_ - distance) * u_x;
        auto y_new = y_robot + (obs_r + collision_radius_ - distance) * u_y;
        turtlelib::Transform2D tr{turtlelib::Vector2D{x_new, y_new}, theta_robot};
        diff->change_transform(tr);
        return;
      }
    }
  }

};

/// \brief The main function to spin the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;

}
