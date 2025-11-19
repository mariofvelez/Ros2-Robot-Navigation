#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	rclcpp::Node::SharedPtr nodeh = rclcpp::Node::make_shared("testnode");

	auto pub = nodeh->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

	geometry_msgs::msg::Twist msg{};
	msg.linear.x = 1;
	
	while (rclcpp::ok())
	{
		pub->publish(msg);
	}

	rclcpp::shutdown();
	return 0;
}
