#pragma once

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_msgs/msg/transition.hpp"


#include "gps2.cpp"

using sensor_msgs::msg::NavSatFix;

struct gpsData
{
	float latitude;
	float longitude;
	float altitude;
};

class gpsLifecyclePublisherNode : public rclcpp_lifecycle::LifecycleNode {
	public:
		explicit gpsLifecyclePublisherNode(const std::string& gpsTopic = "/gps_data");

 	private:
		void publishGpsData();
		gpsData extractGpsData();
		const float MULTIPLYING_FACTOR = 1e-7;

		// std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
		// std::shared_ptr<rclcpp::TimerBase> timer_;

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<NavSatFix>::SharedPtr publisher_;
		SAM_M8Q_GPS gps_;

};
