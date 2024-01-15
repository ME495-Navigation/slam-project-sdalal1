#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
class nusim : public rclcpp::Node
{
    public:
        nusim()
        : Node("nusim"), count_(0)
        {
            timer_ = this->create_wall_timer(
                500ms, std::bind(&nusim::timer_callback, this)
            );
        }
    private:
    void timer_callback(){
        RCLCPP_INFO(this->get_logger(), "Gettting here");
    }
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusim>());
  rclcpp::shutdown();
  return 0;
}