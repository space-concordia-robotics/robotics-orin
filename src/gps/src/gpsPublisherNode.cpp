#include "./gpsPublisherNode.h"

gpsLifecyclePublisherNode::gpsLifecyclePublisherNode(const std::string& gpsTopic) : LifecycleNode("gpsLifecyclePublisherNode")
{
	publisher_ = this->create_publisher<NavSatFix>(gpsTopic, 10);
	RCLCPP_INFO(this->get_logger(), "Started publishing GPS data to %s", gpsTopic.c_str());
	timer_ =
		this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&gpsLifecyclePublisherNode::publishGpsData, this));
}

void gpsLifecyclePublisherNode::publishGpsData()
{
	auto message = sensor_msgs::msg::NavSatFix();

	message.header = std_msgs::msg::Header();
	gpsData data = extractGpsData();
	message.latitude = data.latitude;
	message.longitude = data.longitude;
	message.altitude = data.altitude;

	publisher_->publish(message);
}

gpsData gpsLifecyclePublisherNode::extractGpsData()
{
	char res[256];
	int32_t latitude, longitude, height;
	gps_.pollNAV_PVT(res, latitude, longitude, height);
	gpsData data{};
	data.latitude = float(latitude) * MULTIPLYING_FACTOR;
	data.longitude = float(longitude) * MULTIPLYING_FACTOR;
	data.altitude = float(height) * MULTIPLYING_FACTOR;
	return data;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor exe;

	std::shared_ptr<gpsLifecyclePublisherNode> gps_node = std::make_shared<gpsLifecyclePublisherNode>();
	exe.add_node(gps_node->get_node_base_interface());

	exe.spin();
	rclcpp::shutdown();
	return 0;
}