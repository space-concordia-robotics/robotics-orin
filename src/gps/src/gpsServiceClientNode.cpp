#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

// which node to handle
static constexpr char const * lifecycle_node = "gpsLifecyclePublisherNode";
static constexpr char const * node_get_state_topic = "gpsLifecyclePublisherNode/get_state";
static constexpr char const * node_change_state_topic = "gpsLifecyclePublisherNode/change_state";

template<typename FutureT, 
    typename WaitTimeT> std::future_status wait_for_result(
    FutureT & future,
    WaitTimeT time_to_wait){
        auto end = std::chrono::steady_clock::now() + time_to_wait;
        std::chrono::milliseconds wait_period(100);
        std::future_status status = std::future_status::timeout;
        do {
            auto now = std::chrono::steady_clock::now();
            auto time_left = end - now;
            if (time_left <= std::chrono::seconds(0)) {break;}
            status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
        } while (rclcpp::ok() && status != std::future_status::ready);
        return status;
}

class LifecycleServiceClient : public rclcpp::Node{
    public:
        explicit LifecycleServiceClient(const std::string & node_name)
        : Node(node_name)
        {}

        void init(){
            client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
                node_get_state_topic
            );
            client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
                node_change_state_topic
            );
        }

        unsigned int get_state(std::chrono::seconds time_out = 3s){
            auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

            if (!client_get_state_->wait_for_service(time_out)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Service %s is not available.",
                    client_get_state_->get_service_name());
                return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
            }

            auto future_result = client_get_state_->async_send_request(request).future.share();

            auto future_status = wait_for_result(future_result, time_out);

            if (future_status != std::future_status::ready) {
                RCLCPP_ERROR(
                    get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
                return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
            }

            if (future_result.get()) {
                RCLCPP_INFO(
                    get_logger(), "Node %s has current state %s.",
                    lifecycle_node, future_result.get()->current_state.label.c_str());
                return future_result.get()->current_state.id;
            } else {
                RCLCPP_ERROR(
                    get_logger(), "Failed to get current state for node %s", lifecycle_node);
                return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
            }
        }

        bool change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s){
            auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            request->transition.id = transition;

            if (!client_change_state_->wait_for_service(time_out)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Service %s is not available.",
                    client_change_state_->get_service_name()
                );
                return false;
            }

            auto future_result = client_change_state_->async_send_request(request).future.share();

            auto future_status = wait_for_result(future_result, time_out);

            if (future_status != std::future_status::ready) {
                RCLCPP_ERROR(
                    get_logger(), "Server time out while getting current state for node %s", lifecycle_node
                );
                return false;
            }

            if (future_result.get()->success) {
                RCLCPP_INFO(
                    get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition)
                );
                return true;
            } else {
                RCLCPP_WARN(
                    get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition)
                );
                return false;
            }
        }

    private:
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};


void callee_script(std::shared_ptr<LifecycleServiceClient> gps_client){
    rclcpp::WallRate time_between_state_changes(0.1);  // 10s

    // configure
    {
        if (!gps_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
            return;
        }
        if (!gps_client->get_state()) {
            return;
        }
    }

    // activate
    {
        time_between_state_changes.sleep();
        if (!rclcpp::ok()) {
            return;
        }
        if (!gps_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
            return;
        }
        if (!gps_client->get_state()) {
            return;
        }
    }

    // deactivate
    {
        time_between_state_changes.sleep();
        if (!rclcpp::ok()) {
            return;
        }
        if (!gps_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
            return;
        }
        if (!gps_client->get_state()) {
            return;
        }
    }

    // activate
    {
        time_between_state_changes.sleep();
        if (!rclcpp::ok()) {
            return;
        }
        if (!gps_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
            return;
        }
        if (!gps_client->get_state()) {
            return;
        }
    }

    // deactivate
    {
        time_between_state_changes.sleep();
        if (!rclcpp::ok()) {
            return;
        }
        if (!gps_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
            return;
        }
        if (!gps_client->get_state()) {
            return;
        }
    }

    // cleanup
    {
        time_between_state_changes.sleep();
        if (!rclcpp::ok()) {
            return;
        }
        if (!gps_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
            return;
        }
        if (!gps_client->get_state()) {
            return;
        }
    }

    // and finally shutdown
    {
        time_between_state_changes.sleep();
        if (!rclcpp::ok()) {
            return;
        }
        if (!gps_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)){
            return;
        }
        if (!gps_client->get_state()) {
            return;
        }
    }
}

void wake_executor(std::shared_future<void> future, rclcpp::executors::SingleThreadedExecutor & exec){
    future.wait();
    exec.cancel();
}

int main(int argc, char ** argv){
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto gps_client = std::make_shared<LifecycleServiceClient>("gps_client");
    gps_client->init();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(gps_client);

    std::shared_future<void> script = std::async(
        std::launch::async,
        std::bind(callee_script, gps_client)
    );
    auto wake_exec = std::async(
        std::launch::async,
        std::bind(wake_executor, script, std::ref(exe))
    );

    exe.spin_until_future_complete(script);

    rclcpp::shutdown();

    return 0;
}