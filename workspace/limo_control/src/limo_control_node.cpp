#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LimoControlNode : public rclcpp::Node {
public:
  LimoControlNode() : Node("limo_control_node"), count_(0) {

    // Declare parameters for the target coordinates (x, y, theta) with default
    // values
    this->declare_parameter("target_x", 0.0);
    this->declare_parameter("target_y", 0.0);
    this->declare_parameter("target_theta", 0.0);

    // Update parameters based on whats passed into the launch file
    target_x = this->get_parameter("target_x").as_double();
    target_y = this->get_parameter("target_y").as_double();
    target_theta = this->get_parameter("target_theta").as_double();

    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&LimoControlNode::callback, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    log_file_.open("target_error.csv");
    log_file_ << "Iterations,Euclidean Error,Angular Error\n";
  }

  ~LimoControlNode() { log_file_.close(); }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    static int num_iterations = 0;

    double current_yaw = get_yaw(msg->pose.pose.orientation);

    double current_euclidean_distance =
        get_euclidean_distance(msg->pose.pose.position);
    double current_angle_to_target =
        get_angle_to_target(msg->pose.pose.position);

    double angle_difference = current_angle_to_target - current_yaw;

    // Since the angle difference can be out of the atan2 range, it can lead to
    // a lot of divergence in values near pi or -pi, so the difference is
    // further normalized using atan2 to lock it between -pi and pi

    double angle_difference_normalized =
        atan2(sin(angle_difference), cos(angle_difference));

    double new_linear_velocity = 0;
    if (!euclidean_error_within_bounds) {
      new_linear_velocity = k_v * current_euclidean_distance;
    }

    double new_angular_velocity = k_w * angle_difference_normalized;

    // Publish new angular and linear velocity

    publish_new_command(new_linear_velocity, new_angular_velocity);

    // Logging the errors to a CSV file that can be plotted later
    log_to_file(num_iterations, current_euclidean_distance,
                angle_difference_normalized);

    num_iterations++;

    if (angle_difference_normalized < degrees_to_radians(0.5) &&
        current_euclidean_distance < 0.01) {
      // We are within the desired error range, shut the node down
      RCLCPP_INFO(this->get_logger(), "Achieved target, shutting down");
      rclcpp::shutdown();
    } else if (current_euclidean_distance < 0.01) {
      // The distance is within bounds, the angle isn't, so we spin 
      // in place until we get within the error bound. In order to
      // get the desired resolution, the gain for angular velocity
      // will be lowered to 0.01 and the flag for that will be
      // accordingly set
      euclidean_error_within_bounds = true;
      k_w = 0.1;
      RCLCPP_INFO(this->get_logger(), "Angular Error: {%f}", angle_difference_normalized);
    }
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      publisher_; // The publisher object
  size_t count_;

  // Target variables
  double target_x = 0.0;
  double target_y = 0.0;
  double target_theta = 0.0; // yaw

  // Proportionality constants to determine output linear and angular velocity

  double k_v =
      0.1; // TODO (Populate), proportionality constant for linear velocity
  double k_w =
      0.1; // TODO (Populate), proportionality constant for angular velocity

  std::ofstream log_file_;

  // Flag for when euclidean error is within bounds
  bool euclidean_error_within_bounds = false;

  // Helper functions

  double get_yaw(geometry_msgs::msg::Quaternion &quaternion) {
    // Gets the yaw from the quaternion representing pose/orientation
    tf2::Quaternion quaternion_instance;
    tf2::fromMsg(quaternion, quaternion_instance);
    double roll, pitch, yaw = 0.0;
    tf2::Matrix3x3(quaternion_instance).getRPY(roll, pitch, yaw);
    return yaw;
  }

  double get_euclidean_distance(geometry_msgs::msg::Point &current_position) {
    // Use target variables updated from parameters

    double current_x = current_position.x;
    double current_y = current_position.y;

    double x_diff = (target_x - current_x);
    double y_diff = (target_y - current_y);

    double distance = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    return distance;
  }

  double get_angle_to_target(geometry_msgs::msg::Point &current_position) {
    // Use target variables updated from parameters

    double current_x = current_position.x;
    double current_y = current_position.y;

    double x_diff = (target_x - current_x);
    double y_diff = (target_y - current_y);

    double angle = atan2(y_diff, x_diff);

    return angle;
  }

  void publish_new_command(double linear_vel, double angular_vel) {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = linear_vel;
    message.angular.z = angular_vel;

    publisher_->publish(message);
  }

  void log_to_file(int num_iterations, double euclidean_error, double angular_error) {
    log_file_ << num_iterations << "," << euclidean_error << ","
              << angular_error << "\n";
  }

  double degrees_to_radians(double radians) { return radians * (M_PI / 180.0); }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LimoControlNode>());
  rclcpp::shutdown();
  return 0;
}