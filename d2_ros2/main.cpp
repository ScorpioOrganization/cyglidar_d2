#include "d2_node.h"
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

static std::atomic<bool> g_shutdown_requested{false};
static D2Node* g_d2_node_ptr = nullptr;

static void signal_handler(int /*signum*/)
{
    g_shutdown_requested = true;
    if (g_d2_node_ptr)
        g_d2_node_ptr->cancelSerialRead();
}

int main(int argc, char **argv)
{
    // Disable rclcpp's built-in signal handler so it does not call
    // rcl_shutdown() on SIGINT — we manage shutdown order ourselves
    // to ensure the close frame is published before the context dies.
    rclcpp::init(argc, argv);

    std::shared_ptr<D2Node> d2_node = std::make_shared<D2Node>();
    g_d2_node_ptr = d2_node.get();

    // Override rclcpp's SIGINT handler with ours (must be after rclcpp::init)
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    try
    {
        d2_node->connectBoostSerial();
        rclcpp::Rate rate(1000);
        while (!g_shutdown_requested)
        {
            d2_node->loopCygParser();
            rate.sleep();
        }

        // rclcpp context is still valid here — publish the close frame
        d2_node->disconnectBoostSerial();

        // Give DDS time to deliver the close command before shutdown
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    catch (const rclcpp::exceptions::RCLError& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[D2 NODE ERROR] : %s", e.what());
    }

    g_d2_node_ptr = nullptr;
    rclcpp::shutdown();
}
