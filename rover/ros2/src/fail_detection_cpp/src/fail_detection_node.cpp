#include <fail_detection_cpp/fail_detection.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
    auto fail_detection_node = std::make_shared<FailDetector>(options);

    executor->add_node(fail_detection_node);
    const int node_period = getEnv("FAIL_DETECTION_NODE_PERIOD", 50);

    auto period = std::chrono::milliseconds(node_period);
    rclcpp::Rate r(period);

    while (rclcpp::ok())
    {
        executor->spin_some();
        r.sleep();
    }

    rclcpp::shutdown();

    return 0;
}