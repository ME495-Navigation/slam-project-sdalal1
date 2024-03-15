/// \file landmarks.cpp
/// \brief Obtain the landmarks from the lidar and publish them
///
/// PARAMETERS:
///
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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"

using namespace std::chrono_literals;

/// \brief  Landmarks node implements the main turtlebot functionality using DiffDrive class.
class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    declare_parameter("laser_id", "green/base_scan");

    laser_id = get_parameter("laser_id").as_string();
    lidar_subscription = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), std::bind(&Landmarks::lidar_callback, this, std::placeholders::_1));
    // lidar_subscription = create_subscription<sensor_msgs::msg::LaserScan>(
    //   "scan", 10, std::bind(&Landmarks::lidar_callback, this, std::placeholders::_1));

    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    publish_landmarks = create_publisher<visualization_msgs::msg::MarkerArray>("landmarks",qos_profile);
    publish_fittings = create_publisher<visualization_msgs::msg::MarkerArray>("fitting", qos_profile);


    timer_ = create_wall_timer(
      1000ms / 200, std::bind(&Landmarks::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publish_landmarks;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publish_fittings;
  double laser_minimum_range_, laser_maximum_range_, laser_angle_increment_;
  std::vector<std::vector< turtlelib::Point2D>> clusters;
  visualization_msgs::msg::MarkerArray landmark_array;
  visualization_msgs::msg::Marker marker;
  double a, b, r;
  visualization_msgs::msg::MarkerArray fitting_array;
  visualization_msgs::msg::Marker fitting_marker;
  std::string laser_id;


  /// \brief Callback function for the timer
  void timer_callback()
  {

  }

  /// \brief Callback function for the lidar to create a cluster
  /// \param msg the lidar message
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Extract LiDAR parameters
    double laser_minimum_range = msg->range_min;
    double laser_maximum_range = msg->range_max;
    double laser_angle_increment = msg->angle_increment;

    // Vector to store clusters
    std::vector<std::vector<turtlelib::Point2D>> clusters;

    // Loop through all points in the LiDAR scan
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        // Convert polar coordinates to Cartesian coordinates
        double angle = msg->angle_min + i * laser_angle_increment;
        double range = msg->ranges[i];

        // Check if the point is valid
        if (range < laser_minimum_range || range > laser_maximum_range)
        {
            continue; // Skip invalid points
        }

        turtlelib::Point2D pt;
        pt.x = range * cos(angle);
        pt.y = range * sin(angle);

        // Flag to check if the point is added to any existing cluster
        bool added = false;

        // Iterate through existing clusters to find if the point belongs to any
        for (auto& cluster : clusters)
        {
            // Check if the point is close enough to the cluster
            for (const auto& cluster_point : cluster)
            {
                double dist = sqrt(pow(pt.x - cluster_point.x, 2) + pow(pt.y - cluster_point.y, 2));
                if (dist < 0.10) // Adjust this threshold as needed
                {
                    cluster.push_back(pt);
                    added = true;
                    break;
                }
            }
        }

        // If the point doesn't belong to any existing cluster, create a new cluster
        if (!added)
        {
            clusters.push_back(std::vector<turtlelib::Point2D>{pt});
        }
    }

    
    // Remove clusters smaller than 3 points
    clusters.erase(remove_if(clusters.begin(), clusters.end(),
                         [](const std::vector<turtlelib::Point2D>& cluster) { return cluster.size() < 10 || cluster.size() > 30; }),
               clusters.end());
    // clusters.erase(remove_if(clusters.begin(), clusters.end(),
    //                      [](const std::vector<turtlelib::Point2D>& cluster) { return cluster.size() < 4 || cluster.size() > 20; }),
    //            clusters.end());
    // clusters.erase(remove_if(clusters.begin(), clusters.end(),
    //                      [](const std::vector<turtlelib::Point2D>& cluster) { return cluster.size() < 4; }),
    //            clusters.end());


    RCLCPP_INFO_STREAM(this->get_logger(), "Number of clusters: " << clusters.size());
    // for (const auto& cluster : clusters)
    for (size_t i = 0; i < clusters.size(); i++)
    {
        marker.header.frame_id = laser_id;
        marker.header.stamp = this->get_clock()->now();
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.color.r = 1.0;
        marker.color.a = 1.0;

        for (const auto& point : clusters.at(i))
        {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            marker.points.push_back(p);
            // marker.points.push_back(point);
        }

        landmark_array.markers.push_back(marker);
    }

    // find the circle fitting with the clusters
    for (size_t i = 0; i < clusters.size(); i++)
    {
        const int num_points = static_cast<int> (clusters.at(i).size()); 
        // find the centroid of the cluster
        double x = 0;
        double y = 0;
        for (const auto& point : clusters.at(i))
        {
            x += point.x;
            y += point.y;
        }
        x = x / num_points;
        y = y / num_points;

        std::vector<turtlelib::Point2D> shifted_points;
        double z = 0.0;
        arma::mat data_matrix(num_points,4);

        for (int j = 0; j < num_points; j++)
        {
            turtlelib::Point2D shifted_point;
            shifted_point.x = clusters.at(i).at(j).x - x;
            shifted_point.y = clusters.at(i).at(j).y - y;
            shifted_points.push_back(shifted_point);
            data_matrix(j,0) = shifted_point.x * shifted_point.x + shifted_point.y * shifted_point.y;
            data_matrix(j,1) = shifted_point.x;
            data_matrix(j,2) = shifted_point.y;
            data_matrix(j,3) = 1.0;
            z += std::pow(shifted_point.x, 2) + std::pow(shifted_point.y, 2);
        }
        
        double z_bar = z / num_points;
        arma::mat moment_matrix = data_matrix.t() * data_matrix / num_points;

        arma::mat H = arma::mat(4,4 , arma::fill::zeros);
        H(0,0) = 8.0 * z_bar;
        H(0,3) = 2.0;
        H(1,1) = 1.0;
        H(2,2) = 1.0;
        H(3,0) = 2.0;

        arma::mat H_inv = arma::mat(4,4 , arma::fill::zeros);
        H_inv(0,3) = 1.0/2.0;
        H_inv(1,1) = 1.0;
        H_inv(2,2) = 1.0;
        H_inv(3,0) = 1.0/2.0;
        H_inv(3,3) = -2.0 * z_bar;

        arma::mat U;
        arma::vec s;
        arma::mat V;

        arma::svd(U,s,V,data_matrix);

        arma::mat A;
        arma::mat A_star;


        if (s(3) < 1e-12)
        {
            A = V.col(3);
        }
        else
        {
          arma::mat Y = V * arma::diagmat(s) * V.t();
          arma::mat Q = Y * H_inv * Y;
          arma::mat eigvec;
          arma::vec eigval;
          arma::eig_sym(eigval, eigvec, Q);

          //smallest possible positve eigenvalue and set the currosponsding eigenvector as A
          double min_eigval = 1e20;
          int min_index = 0;
          for (size_t j = 0; j < eigval.size(); j++)
          {
              if (eigval(j) > 0 && eigval(j) < min_eigval)
              {
                  min_eigval = eigval(j);
                  min_index = j;
              }
          }
          A_star = eigvec.col(min_index);
          A = Y.i() * A_star;
        }

        // RCLCPP_INFO_STREAM(this->get_logger(), "A: " << A);

        a = -A(1) / (2 * A(0));
        b = -A(2) / (2 * A(0));
        r = std::sqrt( (A(1) * A(1) + A(2) * A(2) - 4 * A(0) * A(3)) / (4 * A(0) * A(0)) );

        a = a + x;
        b = b + y;

        // RCLCPP_INFO_STREAM(this->get_logger(), "Circle center: " << a << " " << b << " " << r);
        if (fabs(r-0.038) < 0.07)
        {
        publish_fitting(i);
        }
        
    }
    
    
    publish_landmarks -> publish(landmark_array);
    marker.points.clear();
    landmark_array.markers.clear();
    clusters.clear();
    //check if the circle fittings are not very close to each other
    //if they are close, remove the smaller one
    for (size_t i = 0; i < fitting_array.markers.size(); i++)
    {
        for (size_t j = i+1; j < fitting_array.markers.size(); j++)
        {
            double dist = distance(fitting_array.markers.at(i).pose.position.x, fitting_array.markers.at(i).pose.position.y, fitting_array.markers.at(j).pose.position.x, fitting_array.markers.at(j).pose.position.y);
            if (dist < 0.09)
            {
                if (fitting_array.markers.at(i).scale.x < fitting_array.markers.at(j).scale.x)
                {
                    fitting_array.markers.erase(fitting_array.markers.begin() + i);
                }
                else
                {
                    fitting_array.markers.erase(fitting_array.markers.begin() + j);
                }
            }
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Number of fittings: " << fitting_array.markers.size());
    publish_fittings -> publish(fitting_array);
    fitting_array.markers.clear();
}

  void publish_fitting(int id){
    //publish sphere with the fitting
    fitting_marker.header.frame_id = laser_id;
    fitting_marker.header.stamp = this->get_clock()->now();
    fitting_marker.type = visualization_msgs::msg::Marker::SPHERE;
    fitting_marker.action = visualization_msgs::msg::Marker::ADD;
    fitting_marker.id = id;
    fitting_marker.pose.position.x = a;
    fitting_marker.pose.position.y = b;
    fitting_marker.pose.position.z = 0;
    fitting_marker.ns = "fitting";
    fitting_marker.scale.x = 2 * r;
    fitting_marker.scale.y = 2 * r;
    fitting_marker.scale.z = 2 * r;
    fitting_marker.color.g = 1.0;
    fitting_marker.color.r = 1.0;
    fitting_marker.color.b = 1.0;
    fitting_marker.color.a = 1.0;
    fitting_array.markers.push_back(fitting_marker);
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

  /// \brief convert the polar coordinates to cartesian
  /// \param r the radius
  /// \param theta the angle
  turtlelib::Point2D polar_to_cartesian(double r, double theta){
    turtlelib::Point2D pt;
    pt.x = r * std::cos(theta);
    pt.y = r * std::sin(theta);
    return pt;
  }

};

/// \brief The main function to spin the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;

}
