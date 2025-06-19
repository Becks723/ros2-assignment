#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "std_msgs/msg/string.hpp"

#include "node/include.hpp"
#include "node/parser.hpp"

using namespace rclcpp;
using namespace std;
using namespace drivers::common;
using namespace drivers::serial_driver;
using namespace std::chrono_literals;

/*
  节点职责：
  1. 订阅另一节点发布的话题消息，并发布到串口
  2. 接受串口消息，并发布到话题中
  3. 消息需要有帧头和帧尾 头0xAA 尾0x55
*/
class SerialNode : public Node
{
public:
    SerialNode()
        : Node("serialnode_becks723")
    {
        // 初始化subscription和 publisher
        m_sub = create_subscription<std_msgs::msg::String>(TOPIC_ID, 10, 
            std::bind(&SerialNode::subCallback, this, std::placeholders::_1)
        );
        m_pub = create_publisher<std_msgs::msg::String>("topic_becks723", 10);

        const string device = "/dev/ttyUSB0";
        SerialPortConfig config(
            9600,
            FlowControl::NONE,
            Parity::NONE,
            StopBits::ONE
        );

        try
        {
            m_iocontext = std::make_shared<IoContext>(1);

            m_driver = std::make_shared<SerialDriver>(*m_iocontext);
            m_driver->init_port(device, config);
            m_driver->port()->open();

            RCLCPP_INFO(get_logger(), "Serial port initialized successfully.");
            RCLCPP_INFO(get_logger(), "Using device: %s", m_driver->port().get()->device_name().c_str());
            RCLCPP_INFO(get_logger(), "Baud_rate: %d", config.get_baud_rate());
        }
        catch(const std::exception& ex)
        {
            RCLCPP_ERROR(get_logger(), "Failed to initalize serial port: %s", ex.what());
            return;
        }
        
        /*m_timer = create_wall_timer(
            5ms,
            std::bind(&SerialNode::transmitCallback, this)
        );*/

        asyncReceiveSerial();
    }   

private:
    void subCallback(const std_msgs::msg::String& msg)
    {
        RCLCPP_INFO(get_logger(), "I heard: %s", msg.data.c_str());

        m_parser.encode(msg, m_tbuffer);

        vector<uint8_t> transbuf(m_tbuffer.data.begin(), m_tbuffer.data.end());

        auto port = m_driver->port();
        try
        {
            size_t transBytes = port->send(transbuf);
            RCLCPP_INFO(get_logger(), "Transmitted to serial: %s (%ld bytes)", msg.data.c_str(), transBytes);
        }
        catch(const std::exception& ex)
        {
            RCLCPP_ERROR(get_logger(), "Error Transmiting from serial port:%s",ex.what());
        }
    }

    void transmitCallback()
    {

    }

    void asyncReceiveSerial()
    {
        auto port = m_driver->port();
        port->async_receive([this](const vector<uint8_t>& data, const size_t& size)
        {
            message msg;
            msg.data = string(data.begin(), data.end());

            RCLCPP_INFO(get_logger(), "Received from serial: %s", msg.data.c_str());

            m_parser.decode(msg, m_rbuffer);
            m_pub->publish(m_rbuffer);

            RCLCPP_INFO(get_logger(), "I sent: %s", m_rbuffer.data.c_str());

            asyncReceiveSerial();
        });
    }

    Subscription<std_msgs::msg::String>::SharedPtr m_sub;
    Publisher<std_msgs::msg::String>::SharedPtr m_pub;

    TimerBase::SharedPtr m_timer;
    shared_ptr<IoContext> m_iocontext;
    shared_ptr<SerialDriver> m_driver;
    message m_tbuffer;   // 发送缓冲区
    message m_rbuffer;   // 接收缓冲区

    Parser m_parser;
};

int main(int argc, char** argv)
{
    init(argc, argv);
    spin(std::make_shared<SerialNode>());
    shutdown();
    return 0;
}