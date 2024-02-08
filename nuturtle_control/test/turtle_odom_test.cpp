#include <chrono>

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using namespace std::chrono_literals;
rclcpp::Time buffer_time = rclcpp::Clock().now();
rclcpp::Time buffer_time1 = rclcpp::Clock().now();


geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();
geometry_msgs::msg::TransformStamped t1 = geometry_msgs::msg::TransformStamped();

sensor_msgs::msg::JointState js_pub = sensor_msgs::msg::JointState();
sensor_msgs::msg::JointState js_pub1 = sensor_msgs::msg::JointState();

// auto listener = std::make_unique<tf2_ros::TransformListener>(buffer);

TEST_CASE("test for publish","[js_publish_test]"){
auto node = rclcpp::Node::make_shared("turtle_odom_test");


auto publisher_js = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
js_pub.position = std::vector<double>(2);
js_pub.position.at(0) = 0.0;
js_pub.position.at(1) = 0.0;


// auto client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");
auto buffer =std::make_unique<tf2_ros::Buffer>(node->get_clock());
auto listener = std::make_unique<tf2_ros::TransformListener>(*buffer, node);

// auto request = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
// request->x = 0.0, request->y=0.0, request->theta=0.0;

rclcpp::Time start_time = rclcpp::Clock().now();

rclcpp::Time ti = rclcpp::Time(0);

// while (!client->wait_for_service(0s)){
    
//     if(!rclcpp::ok()){
//       std::cout<<"waiting on service"<<std::endl;
//     }
//     }
//     auto future_result = client->async_send_request(request);

//     if (rclcpp::spin_until_future_complete(node, future_result) ==
//     rclcpp::FutureReturnCode::SUCCESS)
//     {
      
//     std::cout<<"success"<<std::endl;
//     } else {
//     std::cout<<"failed to send command"<<std::endl;
//     }

while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < 10s)
  )
  {
    rclcpp::spin_some(node);
    // js_pub.header.stamp = rclcpp::Clock().now();
    js_pub.header.stamp = ti;

    publisher_js -> publish(js_pub);
    try{
    t = buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex){ std::cout<<ex.what()<<std::endl;
    continue;}
    // std::cout<<t.transform.translation.y<<std::endl;
  }

  CHECK_THAT(t.transform.translation.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
  CHECK_THAT(t.transform.translation.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
  CHECK_THAT(t.transform.rotation.z, Catch::Matchers::WithinAbs(0.0, 1e-5));
  CHECK_THAT(t.transform.rotation.w, Catch::Matchers::WithinAbs(1.0, 1e-5));
}


TEST_CASE("service test for initial pose","[inital_pose_test]"){
auto node = rclcpp::Node::make_shared("turtle_odom_test");


auto publisher_js = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
js_pub.position = std::vector<double>(2);
js_pub.position.at(0) = 1.0;
js_pub.position.at(1) = 0.0;

// js_pub1.position = std::vector<double>(2);
// js_pub1.position.at(0) = 2.0;
// js_pub1.position.at(1) = 2.0;


auto client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");
auto buffer =std::make_unique<tf2_ros::Buffer>(node->get_clock());
auto listener = std::make_unique<tf2_ros::TransformListener>(*buffer, node);

auto request = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
request->x = 1.0, request->y=0.5, request->theta=0.5;

rclcpp::Time start_time = rclcpp::Clock().now();
rclcpp::Time ti = rclcpp::Time(0);
js_pub.header.stamp = ti;
// js_pub1.header.stamp = ti + 1s;

publisher_js -> publish(js_pub);
// publisher_js -> publish(js_pub1);

while (!client->wait_for_service(0s)){
    
    if(!rclcpp::ok()){
      std::cout<<"waiting on service"<<std::endl;
    }
    }
    auto future_result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future_result) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
    std::cout<<"success"<<std::endl;
    } else {
    std::cout<<"failed to send command"<<std::endl;
    }

while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < 10s)
  )
  {
    rclcpp::spin_some(node);
    try{
    t = buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex){ std::cout<<ex.what()<<std::endl;
    continue;}
    // std::cout<<t.transform.translation.y<<std::endl;
  }

  CHECK_THAT(t.transform.translation.x, Catch::Matchers::WithinAbs(0.0163832661, 1e-5));
  CHECK_THAT(t.transform.translation.y, Catch::Matchers::WithinAbs(-0.0016955391, 1e-5));
  CHECK_THAT(t.transform.rotation.z, Catch::Matchers::WithinAbs(-0.1029423121, 1e-5));
  CHECK_THAT(t.transform.rotation.w, Catch::Matchers::WithinAbs(0.9946873279, 1e-5));
}