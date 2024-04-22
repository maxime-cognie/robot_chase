#include <cmath>
#include <cstddef>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/impl/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class RobotChase 
    :public rclcpp::Node 
{
public:
    RobotChase() 
        :Node("robot_chase_node") 
    {
        using namespace std::placeholders;

        publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 1);

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ =
            this->create_wall_timer(100ms,
                std::bind(&RobotChase::send_command, this)); 
 
        RCLCPP_INFO(this->get_logger(), "The chase has begun!!!");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    double current_yaw;

    void send_command() 
    {
        geometry_msgs::msg::TransformStamped t;
        std::string origin_frame = "rick/base_link";
        std::string dest_frame = "morty/base_link";

        try {
          t = tf_buffer_->lookupTransform(
            origin_frame, dest_frame,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            origin_frame.c_str(), dest_frame.c_str(), ex.what());
          return;
        }

        double error_distance;
        double error_yaw;

        error_distance = sqrt(
            t.transform.translation.x * t.transform.translation.x +
            t.transform.translation.y * t.transform.translation.y);
        
        error_yaw = atan2(
            t.transform.translation.y,
            t.transform.translation.x);

        geometry_msgs::msg::Twist cmd_vel;
        const double kp_distance = 0.2;
        const double kp_yaw = 2.0 / M_PI;
        
        cmd_vel.linear.x = (kp_distance * error_distance > 1.5)?
            1.5 : kp_distance * error_distance;
        cmd_vel.angular.z = kp_yaw * error_yaw;

        publisher_->publish(cmd_vel);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RobotChase>());
    rclcpp::shutdown();

    return 0;
}