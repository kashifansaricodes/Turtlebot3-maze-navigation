#pragma once

#include "geometry_msgs/msg/twist.hpp"

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>
// for static broadcaster
#include "tf2_ros/static_transform_broadcaster.h"
// for dynamic broadcaster
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
// #include "mage_msgs/msg/part.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <cmath>

using namespace std::chrono_literals;

/**
 * @class BroadcasterDemo
 * @brief A class that represents a broadcaster demo.
 * 
 * This class inherits from rclcpp::Node and provides functionality for broadcasting transforms and handling subscriptions.
 */
class BroadcasterDemo : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for BroadcasterDemo.
     * @param node_name The name of the node.
     */
    BroadcasterDemo(std::string node_name) : Node(node_name)
    {
        // parameter to decide whether to execute the broadcaster or not
        RCLCPP_INFO(this->get_logger(), "Broadcaster demo started");

        // declaring aruco_marker parameters
        this->declare_parameter("aruco_marker_0","right_90");
        this->declare_parameter("aruco_marker_1","left_90");
        this->declare_parameter("aruco_marker_2","end");

        // initialize a static transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);
        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        velocity_timer_= this-> create_wall_timer(std::chrono::milliseconds(1000), std::bind(&BroadcasterDemo::move_robot, this));

        rclcpp::QoS qos(10);     qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        logical_subscription_= this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/advanced_logical_camera/image", 
        qos, std::bind(&BroadcasterDemo::battery_pos, this, std::placeholders::_1));

        aruco_subscription_= this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 
        10, std::bind(&BroadcasterDemo::aruco_pos, this, std::placeholders::_1));

        imu_subscription_= this->create_subscription<sensor_msgs::msg::Imu>("imu", 
        qos, std::bind(&BroadcasterDemo::imu_sensor, this, std::placeholders::_1));

    }

private:

    /*!< Boolean parameter to whether or not start the broadcaster */
    bool param_broadcast_;
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    /*!< Static broadcaster object */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    /*!< Broadcaster object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    //!< Utils object to access utility functions/
    std::shared_ptr<Utils> utils_ptr_;
    //!< Wall timer object for the broadcaster/
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    //!< Wall timer object for the static broadcaster/
    rclcpp::TimerBase::SharedPtr static_broadcast_timer_;

    //flags and intermediate variables
    double marker_id;
    double position_z;


    int selected_aruco;
    int dist_flag{0};
    int marker_set{0};

    double curr_orient{0};
    double init_orient{0};
    int j;

    // Parameter attributes
    std::string aruco_marker_0;
    std::string aruco_marker_1;
    std::string aruco_marker_2;
    //Vectors to store battery location and type
    std::vector<int> battery_color_vector_;    
    std::vector<float> battery_type_vector_;     
    std::vector<float> battery_x_vector_;
    std::vector<float> battery_y_vector_;     
    std::vector<float> battery_z_vector_;    
    std::vector<float> battery_ox_vector_;
    std::vector<float> battery_oy_vector_;  
    std::vector<float> battery_oz_vector_;
    
    rclcpp::TimerBase::SharedPtr velocity_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_subscription_;
    void battery_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription_;
    void aruco_pos(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    void imu_sensor(const sensor_msgs::msg::Imu::SharedPtr msg);

    void move_robot();
    void print_data_vector();

 
};