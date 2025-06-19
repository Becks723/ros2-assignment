#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "node/include.hpp"

using namespace rclcpp;
using namespace std::chrono_literals;

class PublisherNode : public Node
{
public:
    PublisherNode()
        : Node("pubnode_becks723")
    {
        declare_parameter<std::string>(PARAM_TRANSMIT_MSG, "Fake message");
        declare_parameter<int>(PARAM_TRANSMIT_INTERVAL, 1000);

        int interval = get_parameter(PARAM_TRANSMIT_INTERVAL).as_int();

        m_pub = create_publisher<message>(TOPIC_ID, 10);
        m_timer = create_wall_timer(
            std::chrono::milliseconds(interval),
            std::bind(&PublisherNode::transmitCallback, this)
        );
    }

private:
    void transmitCallback()
    {
        auto str = get_parameter(PARAM_TRANSMIT_MSG).as_string();

        message msg;
        msg.data = str + " x" + std::to_string(m_count++);

        m_pub->publish(msg);
        RCLCPP_INFO(get_logger(), "transmitted: %s", msg.data.c_str());
    }

    Publisher<message>::SharedPtr m_pub;
    TimerBase::SharedPtr m_timer;
    size_t m_count; 
};

int main(int argc, char** argv)
{
    init(argc, argv);
    spin(std::make_shared<PublisherNode>());
    shutdown();
    return 0;
}