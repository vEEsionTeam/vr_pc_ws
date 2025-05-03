#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class SphereMover : public rclcpp::Node {
public:
  SphereMover() : Node("sphere_mover") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");

    // Non-blocking wait for service
    timer_wait_ = this->create_wall_timer(
      500ms, std::bind(&SphereMover::check_service_and_start_timer, this));
  }

private:
  void check_service_and_start_timer() {
    if (client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Gazebo service ready.");
      timer_wait_->cancel();  // Stop waiting
      // Start movement loop at ~30 Hz
      timer_ = this->create_wall_timer(
        33ms, std::bind(&SphereMover::move_sphere, this));
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Waiting for /set_entity_state service...");
    }
  }

  void move_sphere() {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_->lookupTransform("global", "imu", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    // Avoid sending if transform hasn't changed
    if (last_stamp_ == transformStamped.header.stamp) {
      return;
    }
    last_stamp_ = transformStamped.header.stamp;

    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = "head_model";
    request->state.pose.position.x = transformStamped.transform.translation.x;
    request->state.pose.position.y = transformStamped.transform.translation.y;
    request->state.pose.position.z = transformStamped.transform.translation.z + 1.5;
    request->state.pose.orientation = transformStamped.transform.rotation;
    request->state.reference_frame = "world";

    // Only send if client is ready
    if (client_->service_is_ready()) {
      client_->async_send_request(request);
    }
  }

  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_wait_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  builtin_interfaces::msg::Time last_stamp_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SphereMover>());
  rclcpp::shutdown();
  return 0;
}
