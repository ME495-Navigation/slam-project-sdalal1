/// \file slam.cpp
/// \brief Contains the implementation of the Slam class
///
/// PARAMETERS:
///     wheel_radius (double): radius of the wheels
///     track_width (double): width between the left and right wheels
///     body_id (string): body frame link name
///     odom_id (string): odom frame link name (defaults: odom)
///     wheel_left (string):  left wheel link name
///     wheel_right (string): right wheel link name
///     input_noise (double): noise in the input
///     sensor_noise (double): noise in the sensor
/// PUBLISHES:
///      /green/path (nav_msgs::msg::Path): contains the path of the green robot
/// SUBSCRIBES:
///      /blue/odom (nav_msgs::msg::Odometry): contains the odometry of the blue robot
///      /fake_sensor (visualization_msgs::msg::MarkerArray): contains the fake sensor data
/// SERVICES:
///      initial_pose (nuturtle_control::srv::InitialPose): Sets the initial pose of green robot
/// BROADCASTS:
///      map -> odom_id
///      map -> base_id

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <armadillo>

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
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"

using namespace std::chrono_literals;

/// \brief  Slam node implements the main turtlebot functionality using DiffDrive class.
class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("body_id", " ");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", " ");
    declare_parameter("wheel_right", " ");
    declare_parameter("input_noise", 1e-5);
    declare_parameter("sensor_noise", 1e-5);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();
    input_noise = get_parameter("input_noise").as_double();
    sensor_noise = get_parameter("sensor_noise").as_double();

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
    // odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_subscriber = create_subscription<nav_msgs::msg::Odometry>(
      "blue/odom", 10, std::bind(&Slam::odom_callback, this, std::placeholders::_1));
    green_path_pub = create_publisher<nav_msgs::msg::Path>("green/path", 10);


    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tr = {};
    diff = std::make_unique<turtlelib::DiffDrive>(tr, track_width_, wheel_radius_);
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    publisher_obs = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/prediction",
      qos_profile);

    rclcpp::QoS qos_profile2(10);
    qos_profile2.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile2.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // marker_subscriber = create_subscription<visualization_msgs::msg::MarkerArray>(
    //   "fake_sensor", 10, std::bind(&Slam::marker_callback, this, std::placeholders::_1));
    circle_fit_subscriber = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fitting", qos_profile2, std::bind(&Slam::fit_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::string body_id = " ";
  std::string odom_id = " ";
  std::string wheel_left = " ";
  std::string wheel_right = " ";
  double wheel_radius_ = 0.0;
  double track_width_ = 0.0;
  double input_noise = 0.0;
  double sensor_noise = 0.0;
  bool first = true;
  sensor_msgs::msg::JointState old_js, js_pos;
  turtlelib::Transform2D tr;
  std::unique_ptr<turtlelib::DiffDrive> diff;
  turtlelib::Transform2D transformation;
  tf2::Quaternion q;
  // nav_msgs::msg::Odometry odom_sub = nav_msgs::msg::Odometry();
  geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();
  geometry_msgs::msg::TransformStamped t1 = geometry_msgs::msg::TransformStamped();

  nav_msgs::msg::Path green_path = nav_msgs::msg::Path();
  geometry_msgs::msg::PoseStamped ps = geometry_msgs::msg::PoseStamped();

  std::vector<double> state;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_path_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_subscriber;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr circle_fit_subscriber;
  double del_x, del_y, del_theta;
  double x_odom, y_odom, theta_odom;
  turtlelib::Transform2D Tmo{};
  turtlelib::Transform2D Tob;
  turtlelib::Transform2D Tmb;
  int max_obs = 10;
  arma::Col<double> state_current{3 + 2 * static_cast<long long unsigned int>(max_obs), arma::fill::zeros};
  arma::Col<double> g_old{3 + 2 * static_cast<long long unsigned int>(max_obs), arma::fill::zeros}; // 3 + 2*max_obs
  std::vector<double> gg;
  arma::Mat<double> sigma_hat_minus;
  turtlelib::Transform2D Tbb_prime;
  turtlelib::Twist2D twist_tbb;
  arma::Mat<double> sigma_zero =
    arma::join_cols(
    arma::join_rows(
      arma::Mat<double>(3, 3, arma::fill::zeros),
      arma::Mat<double>(3, 2 * max_obs, arma::fill::zeros)),
    arma::join_rows(
      arma::Mat<double>(2 * max_obs, 3, arma::fill::zeros),
      arma::Mat<double>(2 * max_obs, 2 * max_obs, arma::fill::eye) * 1e6));
  visualization_msgs::msg::MarkerArray ob = visualization_msgs::msg::MarkerArray();
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_obs;
  arma::Mat<double> A;
  arma::mat R = arma::Mat<double>(2 * max_obs, 2 * max_obs, arma::fill::eye) * 0.0001;
  std::vector<int> seen_ids{};

  /// \brief  Callback function for the marker array
  /// \param msg marker array message
  // void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  // {
    // Tob = turtlelib::Transform2D(turtlelib::Vector2D{x_odom, y_odom}, theta_odom);
    // Tmb = Tmo * Tob;
    // state_current.at(0) = turtlelib::normalize_angle(Tmb.rotation());
    // state_current.at(1) = Tmb.translation().x;
    // state_current.at(2) = Tmb.translation().y;
    // A = var_mat();

    // std::normal_distribution<> d1(0.0, input_noise);

    // // auto q_man = arma::Mat<double>(3, 3, arma::fill::zeros);
    // // q_man(0, 0) = d1(get_random());
    // // q_man(1, 1) = d1(get_random());
    // // q_man(2, 2) = d1(get_random());
    // auto Q =
    //   arma::Mat<double>(
    //   arma::join_cols(
    //     arma::join_rows(
    //       arma::Mat<double>(
    //         3, 3,
    //         arma::fill::eye) * input_noise, arma::Mat<double>(3, 2 * max_obs, arma::fill::zeros)),
    //     arma::join_rows(
    //       arma::Mat<double>(2 * max_obs, 3, arma::fill::zeros),
    //       arma::Mat<double>(2 * max_obs, 2 * max_obs, arma::fill::zeros))));
    // if (first) {
    //   sigma_hat_minus = A * sigma_zero * A.t() + Q;
    //   first = false;
    // } else {
    //   sigma_hat_minus = A * sigma_hat_minus * A.t() + Q;
    // }

    // for (std::size_t i = 0; i < msg->markers.size(); i++) {
    //   if (msg->markers.at(i).id == int(i) &&
    //     !turtlelib::almost_equal(
    //       msg->markers.at(i).pose.position.x, 0.0,
    //       1e-4) && !turtlelib::almost_equal(msg->markers.at(i).pose.position.y, 0.0, 1e-4))
    //   {
    //     turtlelib::Vector2D cyl{msg->markers.at(i).pose.position.x,
    //       msg->markers.at(i).pose.position.y};
    //     auto vec_cyl = cyl;
    //     auto mx = vec_cyl.x;
    //     auto my = vec_cyl.y;
    //     auto id = i;

    //     if (msg->markers.at(i).action != 2) {
    //       calculate_measurement(mx, my, id);
    //     }
    //   }
    // }

    // Tmb = turtlelib::Transform2D(
    //   turtlelib::Vector2D{state_current(1), state_current(
    //       2)}, turtlelib::normalize_angle(state_current(0)));

    // Tmo = Tmb * Tob.inv();
    // t1.header.frame_id = "map";
    // t1.child_frame_id = odom_id;
    // t1.header.stamp = this->get_clock()->now();
    // t1.transform.translation.x = Tmo.translation().x;
    // t1.transform.translation.y = Tmo.translation().y;
    // tf2::Quaternion q_mo;
    // q_mo.setRPY(0, 0, turtlelib::normalize_angle(Tmo.rotation()));
    // t1.transform.rotation.x = q_mo.x();
    // t1.transform.rotation.y = q_mo.y();
    // t1.transform.rotation.z = q_mo.z();
    // t1.transform.rotation.w = q_mo.w();
    // tf_broadcaster_->sendTransform(t1);

    // green_path.header.stamp = this->get_clock()->now();
    // green_path.header.frame_id = "map";
    // ps.header.frame_id = "map";
    // ps.pose.position.x = Tmb.translation().x;
    // ps.pose.position.y = Tmb.translation().y;
    // tf2::Quaternion q_mb;
    // q_mo.setRPY(0, 0, turtlelib::normalize_angle(Tmb.rotation()));
    // ps.pose.orientation.x = q_mb.x();
    // ps.pose.orientation.y = q_mb.y();
    // ps.pose.orientation.z = q_mb.z();
    // ps.pose.orientation.w = q_mb.w();

    // green_path.poses.push_back(ps);
    // green_path_pub->publish(green_path);
    // g_old = state_current;
    // publish_obs();
    // ob.markers.clear();
  // }

  /// \brief  Callback function for the odometry
  /// \param msg odometry message
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    del_x = msg->twist.twist.linear.x;
    del_y = msg->twist.twist.linear.y;
    del_theta = msg->twist.twist.angular.z;
    x_odom = msg->pose.pose.position.x;
    y_odom = msg->pose.pose.position.y;
    auto th = msg->pose.pose.orientation;
    theta_odom = euler_from_quaternion(th);

    twist_tbb = turtlelib::Twist2D{del_theta, del_x, del_y};

    auto Tbb_subprime = turtlelib::integrate_twist(twist_tbb);


    Tbb_prime = Tbb_subprime;
    t.header.frame_id = odom_id;
    t.child_frame_id = body_id;
    t.header.stamp = this->get_clock()->now();
    t.transform.translation.x = x_odom;
    t.transform.translation.y = y_odom;

    tf2::Quaternion q;
    q.setRPY(0, 0, turtlelib::normalize_angle(theta_odom));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);

  }


  void fit_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg){
  //use maha distance to fit the circle
  Tob = turtlelib::Transform2D(turtlelib::Vector2D{x_odom, y_odom}, theta_odom);
  Tmb = Tmo * Tob;
  state_current.at(0) = turtlelib::normalize_angle(Tmb.rotation());
  state_current.at(1) = Tmb.translation().x;
  state_current.at(2) = Tmb.translation().y;
  
  RCLCPP_INFO_STREAM(get_logger(), "marks size"<< msg->markers.size());
  for(size_t i = 0; i < msg->markers.size(); i++){
    //convert x and y to range and bearing
    // RCLCPP_INFO_STREAM(get_logger(), "getting here");
    arma::mat z = arma::mat(2, 1, arma::fill::zeros);
    auto x = msg->markers.at(i).pose.position.x;
    auto y = msg->markers.at(i).pose.position.y;
    auto r = std::sqrt(x*x + y*y);
    auto theta = std::atan2(y,x);
    z(0,0) = r;
    z(1,0) = turtlelib::normalize_angle(theta);
    double thresh_dist = 0.3;
    auto thresh_id = seen_ids.size();
    // RCLCPP_ERROR_STREAM(get_logger(), "seen_ids" << seen_ids.size());
    A = var_mat();

    std::normal_distribution<> d1(0.0, input_noise);
      auto Q =
        arma::Mat<double>(
        arma::join_cols(
          arma::join_rows(
            arma::Mat<double>(
              3, 3,
              arma::fill::eye) * input_noise, arma::Mat<double>(3, 2 * max_obs, arma::fill::zeros)),
          arma::join_rows(
            arma::Mat<double>(2 * max_obs, 3, arma::fill::zeros),
            arma::Mat<double>(2 * max_obs, 2 * max_obs, arma::fill::zeros))));

    if (first) {
      sigma_hat_minus = A * sigma_zero * A.t() + Q;
      first = false;
    } else {
      sigma_hat_minus = A * sigma_hat_minus * A.t() + Q;
    }
    for (size_t j = 0; j < seen_ids.size(); j++){
      auto dx_hat = state_current(3 + 2 * seen_ids[j]) - state_current(1);
      auto dy_hat = state_current(3 + 2 * seen_ids[j] + 1) - state_current(2);
      auto q = std::pow(dx_hat, 2) + std::pow(dy_hat, 2);

      arma::vec z_hat =
      arma::vec(
      {std::sqrt(q),
        turtlelib::normalize_angle(std::atan2(dy_hat, dx_hat) - state_current.at(0))});

      arma::Mat<double> H_i = arma::Mat<double>(2, 3 + 2 * max_obs, arma::fill::zeros);
      H_i.at(0, 0) = 0.0;
      H_i.at(0, 1) = -dx_hat / std::sqrt(q);
      H_i.at(0, 2) = -dy_hat / std::sqrt(q);
      H_i.at(1, 0) = -1.0;
      H_i.at(1, 1) = dy_hat / q;
      H_i.at(1, 2) = -dx_hat / q;
      H_i.at(1, 3 + 2 * j) = -dy_hat / q;
      H_i.at(1, 3 + 2 * j + 1) = dx_hat / q;
      H_i.at(0, 3 + 2 * j) = dx_hat / std::sqrt(q);
      H_i.at(0, 3 + 2 * j + 1) = dy_hat / std::sqrt(q);
      // RCLCPP_INFO_STREAM(get_logger(), "A" << A);
      // RCLCPP_INFO_STREAM(get_logger(), "sigma" << sigma_zero );
      // RCLCPP_INFO_STREAM(get_logger(), "sigma hat minus" << sigma_hat_minus);
      

      arma::Mat<double> Ri = R.submat(2 * j, 2 * j, 2 * j + 1, 2 * j + 1);
      arma::Mat<double> psi = H_i * sigma_hat_minus * H_i.t() + Ri;
      arma::vec zj_new{2, arma::fill::zeros};
      zj_new.at(0) = z(0) - z_hat.at(0);
      zj_new.at(1) = turtlelib::normalize_angle(z(1) - z_hat(1));
      arma::mat mahadist = zj_new.t() * psi.i() * zj_new;
      if (mahadist.at(0) < thresh_dist){
        thresh_dist = mahadist.at(0);
        thresh_id = seen_ids[j];
      } 
    }

    RCLCPP_INFO_STREAM(get_logger(), "thresh_id" << seen_ids.size());
    if(thresh_id == seen_ids.size() && seen_ids.size() < max_obs){
      RCLCPP_INFO_STREAM(get_logger(), "getting here IF");
      seen_ids.push_back(thresh_id);
      state_current(3 + 2 * thresh_id) = state_current(1) + z(0) * std::cos(turtlelib::normalize_angle(state_current(0) + z(1)));
      state_current(3 + 2 * thresh_id + 1) = state_current(2) + z(0) * std::sin(turtlelib::normalize_angle(state_current(0) + z(1)));
      calculate_measurement(x, y, thresh_id);
    }
    if(thresh_id < max_obs){
      RCLCPP_INFO_STREAM(get_logger(), "getting here ELSE");
      calculate_measurement(x, y, thresh_id);
    }
  }
  Tmb = turtlelib::Transform2D(
      turtlelib::Vector2D{state_current(1), state_current(
          2)}, turtlelib::normalize_angle(state_current(0)));

  Tmo = Tmb * Tob.inv();
  t1.header.frame_id = "map";
  t1.child_frame_id = odom_id;
  t1.header.stamp = this->get_clock()->now();
  t1.transform.translation.x = Tmo.translation().x;
  t1.transform.translation.y = Tmo.translation().y;
  tf2::Quaternion q_mo;
  q_mo.setRPY(0, 0, turtlelib::normalize_angle(Tmo.rotation()));
  t1.transform.rotation.x = q_mo.x();
  t1.transform.rotation.y = q_mo.y();
  t1.transform.rotation.z = q_mo.z();
  t1.transform.rotation.w = q_mo.w();
  tf_broadcaster_->sendTransform(t1);

  green_path.header.stamp = this->get_clock()->now();
  green_path.header.frame_id = "map";
  ps.header.frame_id = "map";
  ps.pose.position.x = Tmb.translation().x;
  ps.pose.position.y = Tmb.translation().y;
  tf2::Quaternion q_mb;
  q_mo.setRPY(0, 0, turtlelib::normalize_angle(Tmb.rotation()));
  ps.pose.orientation.x = q_mb.x();
  ps.pose.orientation.y = q_mb.y();
  ps.pose.orientation.z = q_mb.z();
  ps.pose.orientation.w = q_mb.w();

  green_path.poses.push_back(ps);
  green_path_pub->publish(green_path);
  g_old = state_current;
  publish_obs();
  ob.markers.clear();
  RCLCPP_INFO_STREAM(get_logger(), "state" << state_current);
  }



  /// \brief  Getting the twist from the transform
  /// \param Tbb transform
  /// \return twist of the transform
  turtlelib::Twist2D get_twist(turtlelib::Transform2D Tbb)
  {
    if (turtlelib::almost_equal(
        Tbb.translation().y,
        0.0) && turtlelib::almost_equal(Tbb.rotation(), 0.0))
    {
      return turtlelib::Twist2D{0, Tbb.translation().x, 0};
    } else {
      double r = fabs(Tbb.translation().y / (1 - cos(Tbb.rotation())));
      return turtlelib::Twist2D{Tbb.rotation(), r * Tbb.rotation(), 0};
    }

  }

  /// \brief  Convert Quaternion to euler angles
  /// \param quaternion quaternion
  /// \return euler angles yaw
  double euler_from_quaternion(geometry_msgs::msg::Quaternion quaternion)
  {
    auto x = quaternion.x;
    auto y = quaternion.y;
    auto z = quaternion.z;
    auto w = quaternion.w;

    auto siny_cosp = 2 * (w * z + x * y);
    auto cosy_cosp = 1 - 2 * (y * y + z * z);
    auto yaw = std::atan2(siny_cosp, cosy_cosp);

    return yaw;
  }

  /// \brief  Publish the obstacles
  void publish_obs()
  {
    int i = 0;

    for (i = 0; i < max_obs; i++) {
      if (state_current.n_elem > 3 && state_current(3 + 2 * i) != 0.0 &&
        state_current(3 + 2 * i + 1) != 0.0)
      {
        visualization_msgs::msg::Marker cyl = visualization_msgs::msg::Marker();
        cyl.header.stamp = this->get_clock()->now();
        cyl.header.frame_id = "map";
        cyl.id = i + 1;
        cyl.type = visualization_msgs::msg::Marker::CYLINDER;
        cyl.action = visualization_msgs::msg::Marker::ADD;
        cyl.scale.x = 0.05 * 2;
        cyl.scale.y = 0.05 * 2;
        cyl.scale.z = 0.25;
        cyl.pose.position.x = state_current(3 + 2 * i);
        cyl.pose.position.y = state_current(3 + 2 * i + 1);
        cyl.pose.position.z = 0.25 / 2;
        cyl.color.r = 0.0;
        cyl.color.g = 1.0;
        cyl.color.b = 0.0;
        cyl.color.a = 1.0;
        ob.markers.push_back(cyl);
        publisher_obs->publish(ob);
      }
    }
  }

  /// \brief  Return the covariance matrix
  /// \return covariance matrix
  arma::Mat<double> var_mat()
  {
    if (turtlelib::almost_equal(twist_tbb.omega, 0.0)) {
      arma::Mat<double> a_mat = arma::Mat<double>(
        3 + 2 * max_obs, 3 + 2 * max_obs,
        arma::fill::zeros);
      arma::Mat<double> identity = arma::Mat<double>(
        3 + 2 * max_obs, 3 + 2 * max_obs,
        arma::fill::eye);
      a_mat(0, 0) = 0.0;
      // a_mat(1,0) = -state_current(2) + g_old(2);
      // a_mat(2,0) = state_current(1) - g_old(1);
      a_mat(1, 0) = -twist_tbb.x * std::sin(turtlelib::normalize_angle(g_old(0)));
      a_mat(2, 0) = twist_tbb.x * std::cos(turtlelib::normalize_angle(g_old(0)));

      arma::Mat<double> var = identity + a_mat;
      return var;
    } else {
      arma::Mat a_mat = arma::Mat<double>(3 + 2 * max_obs, 3 + 2 * max_obs, arma::fill::zeros);
      arma::Mat<double> identity = arma::Mat<double>(
        3 + 2 * max_obs, 3 + 2 * max_obs,
        arma::fill::eye);
      a_mat(0, 0) = 0.0;
      a_mat(
        1,
        0) = (twist_tbb.x / twist_tbb.omega) *
        (std::cos(turtlelib::normalize_angle(g_old(0) + twist_tbb.omega)) -
        std::cos(turtlelib::normalize_angle(g_old(0))));
      a_mat(
        2,
        0) = (twist_tbb.x / twist_tbb.omega) *
        (std::sin(turtlelib::normalize_angle(g_old(0) + twist_tbb.omega)) -
        std::sin(turtlelib::normalize_angle(g_old(0))));
      arma::Mat<double> var = identity + a_mat;
      return var;
    }
  }

  /// \brief  Calculate the measurement
  /// \param mx x coordinate of the measurement
  /// \param my y coordinate of the measurement
  /// \param id id of the measurement
  void calculate_measurement(double mx, double my, int id)
  {
    auto dx = mx;
    auto dy = my;
    auto d = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    arma::vec zj = arma::vec({d, turtlelib::normalize_angle(std::atan2(dy, dx))});

    if (turtlelib::almost_equal(
        state_current(3 + 2 * id),
        0.0) && turtlelib::almost_equal(state_current(3 + 2 * id + 1), 0.0))
    {
      state_current(3 + 2 * id) = state_current(1) + d * std::cos(state_current(0) + zj(1));
      state_current(3 + 2 * id + 1) = state_current(2) + d * std::sin(state_current(0) + zj(1));
    }

    auto dx_hat = state_current.at(3 + 2 * id) - state_current.at(1);
    auto dy_hat = state_current.at(3 + 2 * id + 1) - state_current.at(2);
    auto q = std::pow(dx_hat, 2) + std::pow(dy_hat, 2);
    arma::vec z_hat =
      arma::vec(
      {std::sqrt(q),
        turtlelib::normalize_angle(std::atan2(dy_hat, dx_hat) - state_current.at(0))});

    arma::Mat<double> H_i = arma::Mat<double>(2, 3 + 2 * max_obs, arma::fill::zeros);
    H_i.at(0, 0) = 0.0;
    H_i.at(0, 1) = -dx_hat / std::sqrt(q);
    H_i.at(0, 2) = -dy_hat / std::sqrt(q);
    H_i.at(1, 0) = -1.0;
    H_i.at(1, 1) = dy_hat / q;
    H_i.at(1, 2) = -dx_hat / q;
    H_i.at(1, 3 + 2 * id) = -dy_hat / q;
    H_i.at(1, 3 + 2 * id + 1) = dx_hat / q;
    H_i.at(0, 3 + 2 * id) = dx_hat / std::sqrt(q);
    H_i.at(0, 3 + 2 * id + 1) = dy_hat / std::sqrt(q);


    arma::Mat<double> Ri = R.submat(2 * id, 2 * id, 2 * id + 1, 2 * id + 1);
    arma::Mat<double> K = sigma_hat_minus * H_i.t() * (H_i * sigma_hat_minus * H_i.t() + Ri).i();
    arma::vec zj_new{2, arma::fill::zeros};
    zj_new.at(0) = zj.at(0) - z_hat.at(0);
    zj_new.at(1) = turtlelib::normalize_angle(zj.at(1) - z_hat.at(1));

    state_current = state_current + K * (zj_new);
    state_current.at(0) = turtlelib::normalize_angle(state_current.at(0));

    auto identity = arma::Mat<double>(3 + 2 * max_obs, 3 + 2 * max_obs, arma::fill::eye);
    sigma_hat_minus = (identity - K * H_i) * sigma_hat_minus;

  }

  /// \brief  Get the random number generator
  std::mt19937 & get_random()
  {
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
  }
};

/// \brief The main function to spin the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;

}
