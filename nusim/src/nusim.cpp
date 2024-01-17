#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/u_int64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;
class Nusim : public rclcpp::Node
{
    public:
        Nusim()
        : Node("nusim")
        {
            declare_parameter("rate",200.0);
            declare_parameter("x0",0.0);
            declare_parameter("y0",0.0);
            declare_parameter("theta0",0.0);

            auto rate = get_parameter("rate").as_double();
            x0 = get_parameter("x0").as_double();
            auto y0 = get_parameter("y0").as_double();
            auto theta0 = get_parameter("theta0").as_double();
            
            timer_ = create_wall_timer(
                1000ms/rate, std::bind(&Nusim::timer_callback, this));
            srv_reset = create_service<std_srvs::srv::Empty>("~/reset", std::bind(&Nusim::reset_cb, this, std::placeholders::_1, std::placeholders::_2));
            srv_teleport = create_service<nusim::srv::Teleport>("~/teleport", std::bind(&Nusim::teleport_cb, this, std::placeholders::_1, std::placeholders::_2)); 
            publisher_timestep = create_publisher<std_msgs::msg::UInt64>("~/timestep",10);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            ground_frame_loc(x0,y0,theta0);
        }
    private:
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr srv_teleport;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_timestep;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();

    double x0 = 0.0;
    double y0 = 0.0;
    double theta0 = 0.0;

    void ground_frame_loc(double x, double y, double theta){
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0,0, theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
    }

    void timer_callback(){
        auto time_msg = std_msgs::msg::UInt64();
        count_++;
        time_msg.data = count_;
        publisher_timestep->publish(time_msg);
        // RCLCPP_INFO_STREAM(get_logger(), "Gettting here "<<  time_msg.data);
    }

    void teleport_cb(const std::shared_ptr<nusim::srv::Teleport::Request> request,
      const std::shared_ptr<nusim::srv::Teleport::Response> response){
        
        ground_frame_loc(request->x, request->y, request->theta);
    }

    void reset_cb(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      const std::shared_ptr<std_srvs::srv::Empty::Response> response){
        count_ = 0;
        ground_frame_loc(x0,y0,theta0);
        RCLCPP_INFO_STREAM(get_logger(),"resetting all variables"<<x0<<y0<<theta0);
    }
    rclcpp::TimerBase::SharedPtr timer_;

    size_t count_{0};

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}    