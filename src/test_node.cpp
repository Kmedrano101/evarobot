// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class myrobot_test : public rclcpp::Node
{
    public:
        myrobot_test()
        : Node("test_node")
        {
            // Perform testing functions from lib frame_transforms
            function_greed();
        }

    private:
        void function_greed()
        {

            RCLCPP_INFO(this->get_logger(), "Hello from python and c++ package");

        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<myrobot_test>());
    rclcpp::shutdown();
    return 0;
}