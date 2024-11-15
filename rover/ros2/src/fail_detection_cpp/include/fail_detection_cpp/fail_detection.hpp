/*! @package fail_detection
    Code Information:
        Maintainer: Eng. Pedro Alejandro Gonzalez B
        Mail: pedro@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <usr_msgs/msg/fails.hpp>
#include <utils/console.hpp>

class FailDetector : public rclcpp::Node
{
   public:
    /*!
        FailDetector Class constructor. Inherits from rclcpp::Node
        @param options: rclcpp::NodeOptions.
    */
    FailDetector(rclcpp::NodeOptions& options);

    /**
     * Receive the imu msg and process it to detect collisions
    @param msg: Imu containing the imu readings
    @return void
     * */
    void ImuCb(sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * Receive the chassis imu msg and process it to detect pitch changes
    @param msg: Imu containing the imu readings
    @return void
     * */
    void ChassisImuCb(sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * Receive the speed command of the robot to know if its going forward or backwards
    @param msg: Twist containing the bot speed command
    @return void
     * */
    void BotSpeedCb(nav_msgs::msg::Odometry::SharedPtr msg);

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_subs_imu_camera;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_subs_imu_chassis;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subs_bot_speed;

    // Publishers
    rclcpp::Publisher<usr_msgs::msg::Fails>::SharedPtr m_pub_fail;

    // Timers
    // Add timers if you need someone
    //rclcpp::TimerBase::SharedPtr m_dummy_tmr;


    // Environment variables
    float m_collision_jerk = getEnv("FAIL_DETECTION_COLLISION_JERK", 400.0f);
    const int m_imu_no_msgs_report_time = getEnv("FAIL_DETECTION_IMU_NO_MSGS_REPORT_TIME", 5);
    const size_t m_n_samples_collision = getEnv("FAIL_DETECTION_COLLISION_SAMPLES", 5);
    const size_t m_n_samples_fall = getEnv("FAIL_DETECTION_COLLISION_SAMPLES", 100);
    const size_t m_n_samples_pitch = getEnv("FAIL_DETECTION_COLLISION_SAMPLES", 15);
    const size_t m_n_samples = std::max(m_n_samples_fall, m_n_samples_collision);

    // Class variables
    double m_bot_speed = 0.0;
    std::string m_motion_state = "forward";

    std::deque<sensor_msgs::msg::Imu> m_imu_msgs_deque = std::deque<sensor_msgs::msg::Imu>(m_n_samples);
    std::deque<float> m_accel_deque = std::deque<float>(m_n_samples);

    usr_msgs::msg::Fails fails_msg;
};