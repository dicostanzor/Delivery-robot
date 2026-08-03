// ELARA CareGo ATR
// Motor control + four-wheel mecanum encoder odometry
//
// Subscribes:
//   /cmd_vel
//
// Publishes:
//   /wheel/odom
//
// Important:
//   This node DOES NOT publish odom -> base_link TF.
//   robot_localization EKF will own that TF later.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "ctre/phoenix/motorcontrol/FeedbackDevice.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace std::chrono_literals;

// ================= ROBOT GEOMETRY =================

// 6-inch mecanum wheels
static constexpr double WHEEL_DIAMETER_M = 0.1524;
static constexpr double WHEEL_CIRCUMFERENCE_M =
    M_PI * WHEEL_DIAMETER_M;

// SRX Magnetic Encoder attached directly to wheel/output shaft
static constexpr double ENCODER_COUNTS_PER_WHEEL_REV = 4096.0;
static constexpr double METERS_PER_ENCODER_COUNT =
    WHEEL_CIRCUMFERENCE_M / ENCODER_COUNTS_PER_WHEEL_REV;

// Wheel-center-to-wheel-center dimensions: 21.5 in × 21.5 in
static constexpr double WHEELBASE_LENGTH_M = 0.5461;
static constexpr double WHEELBASE_WIDTH_M  = 0.5461;

static constexpr double HALF_LENGTH_M =
    WHEELBASE_LENGTH_M / 2.0;
static constexpr double HALF_WIDTH_M =
    WHEELBASE_WIDTH_M / 2.0;

// Mecanum rotational geometry
static constexpr double L_PLUS_W =
    HALF_LENGTH_M + HALF_WIDTH_M;

// Encoder signs.
// We will verify these with one short forward test.
// Change individual values to -1.0 only when required.
static constexpr double FL_ENCODER_SIGN = 1.0;
static constexpr double FR_ENCODER_SIGN = 1.0;
static constexpr double RL_ENCODER_SIGN = 1.0;
static constexpr double RR_ENCODER_SIGN = 1.0;

// ================= MOTOR CONTROL =================

static constexpr double TIMEOUT_MS = 200.0;
static constexpr double MIN_OUTPUT = 0.10;
static constexpr double MAX_OUTPUT = 0.20;

// Preserve the previously calibrated open-loop trims.
static constexpr double FL_TRIM = 1.00;
static constexpr double FR_TRIM = 1.05;
static constexpr double RL_TRIM = 1.15;
static constexpr double RR_TRIM = 1.05;

static constexpr int PID_LOOP_INDEX = 0;
static constexpr int CONFIG_TIMEOUT_MS = 100;

// Odometry update rate
static constexpr double ODOM_PERIOD_SECONDS = 0.02;

// Reject impossible encoder jumps while debugging.
// 1000 counts in 20 ms is far above the intended robot speed.
static constexpr std::int64_t MAX_COUNTS_PER_UPDATE = 1000;

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode()
    : Node("motor_control_node"),
      fl_(3),
      fr_(4),
      rl_(1),
      rr_(2)
    {
        // Previously verified motor inversion.
        fl_.SetInverted(false);  // ID 3
        fr_.SetInverted(true);   // ID 4
        rl_.SetInverted(false);  // ID 1
        rr_.SetInverted(true);   // ID 2

        configure_encoder(fl_, "FL", 3);
        configure_encoder(fr_, "FR", 4);
        configure_encoder(rl_, "RL", 1);
        configure_encoder(rr_, "RR", 2);

        cmd_vel_sub_ =
            create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(
                    &MotorControlNode::cmd_vel_callback,
                    this,
                    std::placeholders::_1));

        wheel_odom_pub_ =
            create_publisher<nav_msgs::msg::Odometry>(
                "/wheel/odom",
                20);

        control_timer_ =
            create_wall_timer(
                10ms,
                std::bind(
                    &MotorControlNode::control_loop,
                    this));

        odom_timer_ =
            create_wall_timer(
                20ms,
                std::bind(
                    &MotorControlNode::odometry_loop,
                    this));

        last_cmd_time_ = now();
        last_odom_time_ = now();

        initialize_encoder_positions();

        RCLCPP_INFO(
            get_logger(),
            "Motor control + wheel odometry started.");

        RCLCPP_INFO(
            get_logger(),
            "Publishing /wheel/odom; not publishing TF.");

        RCLCPP_INFO(
            get_logger(),
            "Wheel diameter %.4f m, meters/count %.9f, L+W %.4f m",
            WHEEL_DIAMETER_M,
            METERS_PER_ENCODER_COUNT,
            L_PLUS_W);
    }

private:
    TalonSRX fl_;
    TalonSRX fr_;
    TalonSRX rl_;
    TalonSRX rr_;

    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<
        nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    geometry_msgs::msg::Twist latest_cmd_;

    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_odom_time_;

    std::int64_t previous_fl_counts_{0};
    std::int64_t previous_fr_counts_{0};
    std::int64_t previous_rl_counts_{0};
    std::int64_t previous_rr_counts_{0};

    double x_{0.0};
    double y_{0.0};
    double yaw_{0.0};

    void configure_encoder(
        TalonSRX & talon,
        const char * name,
        int can_id)
    {
        const auto error =
            talon.ConfigSelectedFeedbackSensor(
                FeedbackDevice::CTRE_MagEncoder_Relative,
                PID_LOOP_INDEX,
                CONFIG_TIMEOUT_MS);

        RCLCPP_INFO(
            get_logger(),
            "%s Talon ID %d encoder configuration result: %d",
            name,
            can_id,
            static_cast<int>(error));
    }

    static std::int64_t pulse_width_position(
        TalonSRX & talon)
    {
        return static_cast<std::int64_t>(
            talon.GetSensorCollection()
                 .GetPulseWidthPosition());
    }

    void initialize_encoder_positions()
    {
        previous_fl_counts_ = pulse_width_position(fl_);
        previous_fr_counts_ = pulse_width_position(fr_);
        previous_rl_counts_ = pulse_width_position(rl_);
        previous_rr_counts_ = pulse_width_position(rr_);

        RCLCPP_INFO(
            get_logger(),
            "Initial encoder counts: FL=%ld FR=%ld RL=%ld RR=%ld",
            static_cast<long>(previous_fl_counts_),
            static_cast<long>(previous_fr_counts_),
            static_cast<long>(previous_rl_counts_),
            static_cast<long>(previous_rr_counts_));
    }

    void cmd_vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_cmd_ = *msg;
        last_cmd_time_ = now();
    }

    static double scale_and_limit(double value)
    {
        if (std::abs(value) < 1e-6) {
            return 0.0;
        }

        double magnitude = std::abs(value);

        if (magnitude < MIN_OUTPUT) {
            magnitude = MIN_OUTPUT;
        }

        if (magnitude > MAX_OUTPUT) {
            magnitude = MAX_OUTPUT;
        }

        return std::copysign(magnitude, value);
    }

    void stop_all()
    {
        fl_.Set(ControlMode::PercentOutput, 0.0);
        fr_.Set(ControlMode::PercentOutput, 0.0);
        rl_.Set(ControlMode::PercentOutput, 0.0);
        rr_.Set(ControlMode::PercentOutput, 0.0);
    }

    void control_loop()
    {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

        const auto current_time = now();

        const double elapsed_ms =
            (current_time - last_cmd_time_)
                .nanoseconds() * 1e-6;

        if (elapsed_ms > TIMEOUT_MS) {
            stop_all();
            return;
        }

        const double vx = latest_cmd_.linear.x;
        const double vy = latest_cmd_.linear.y;
        const double wz = latest_cmd_.angular.z;

        // Mecanum inverse kinematics
        double fl_output = vx - vy - wz * L_PLUS_W;
        double fr_output = vx + vy + wz * L_PLUS_W;
        double rl_output = vx + vy - wz * L_PLUS_W;
        double rr_output = vx - vy + wz * L_PLUS_W;

        const double maximum = std::max({
            std::abs(fl_output),
            std::abs(fr_output),
            std::abs(rl_output),
            std::abs(rr_output),
            1.0
        });

        fl_output /= maximum;
        fr_output /= maximum;
        rl_output /= maximum;
        rr_output /= maximum;

        fl_.Set(
            ControlMode::PercentOutput,
            scale_and_limit(fl_output * FL_TRIM));

        fr_.Set(
            ControlMode::PercentOutput,
            scale_and_limit(fr_output * FR_TRIM));

        rl_.Set(
            ControlMode::PercentOutput,
            scale_and_limit(rl_output * RL_TRIM));

        rr_.Set(
            ControlMode::PercentOutput,
            scale_and_limit(rr_output * RR_TRIM));
    }

    static bool valid_delta(std::int64_t delta)
    {
        return std::llabs(delta) <= MAX_COUNTS_PER_UPDATE;
    }

    void odometry_loop()
    {
        const auto current_time = now();

        double dt =
            (current_time - last_odom_time_)
                .nanoseconds() * 1e-9;

        if (dt <= 0.0 || dt > 0.5) {
            dt = ODOM_PERIOD_SECONDS;
        }

        last_odom_time_ = current_time;

        const std::int64_t current_fl =
            pulse_width_position(fl_);
        const std::int64_t current_fr =
            pulse_width_position(fr_);
        const std::int64_t current_rl =
            pulse_width_position(rl_);
        const std::int64_t current_rr =
            pulse_width_position(rr_);

        std::int64_t delta_fl =
            current_fl - previous_fl_counts_;
        std::int64_t delta_fr =
            current_fr - previous_fr_counts_;
        std::int64_t delta_rl =
            current_rl - previous_rl_counts_;
        std::int64_t delta_rr =
            current_rr - previous_rr_counts_;

        previous_fl_counts_ = current_fl;
        previous_fr_counts_ = current_fr;
        previous_rl_counts_ = current_rl;
        previous_rr_counts_ = current_rr;

        if (!valid_delta(delta_fl) ||
            !valid_delta(delta_fr) ||
            !valid_delta(delta_rl) ||
            !valid_delta(delta_rr))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Rejected encoder jump: FL=%ld FR=%ld RL=%ld RR=%ld",
                static_cast<long>(delta_fl),
                static_cast<long>(delta_fr),
                static_cast<long>(delta_rl),
                static_cast<long>(delta_rr));

            return;
        }

        const double distance_fl =
            FL_ENCODER_SIGN *
            static_cast<double>(delta_fl) *
            METERS_PER_ENCODER_COUNT;

        const double distance_fr =
            FR_ENCODER_SIGN *
            static_cast<double>(delta_fr) *
            METERS_PER_ENCODER_COUNT;

        const double distance_rl =
            RL_ENCODER_SIGN *
            static_cast<double>(delta_rl) *
            METERS_PER_ENCODER_COUNT;

        const double distance_rr =
            RR_ENCODER_SIGN *
            static_cast<double>(delta_rr) *
            METERS_PER_ENCODER_COUNT;

        // Forward mecanum kinematics
        const double delta_x_body =
            (distance_fl +
             distance_fr +
             distance_rl +
             distance_rr) / 4.0;

        const double delta_y_body =
            (-distance_fl +
              distance_fr +
              distance_rl -
              distance_rr) / 4.0;

        const double delta_yaw =
            (-distance_fl +
              distance_fr -
              distance_rl +
              distance_rr) /
            (4.0 * L_PLUS_W);

        const double midpoint_yaw =
            yaw_ + delta_yaw * 0.5;

        x_ +=
            delta_x_body * std::cos(midpoint_yaw) -
            delta_y_body * std::sin(midpoint_yaw);

        y_ +=
            delta_x_body * std::sin(midpoint_yaw) +
            delta_y_body * std::cos(midpoint_yaw);

        yaw_ += delta_yaw;
        yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));

        const double vx = delta_x_body / dt;
        const double vy = delta_y_body / dt;
        const double wz = delta_yaw / dt;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z =
            std::sin(yaw_ * 0.5);
        odom.pose.pose.orientation.w =
            std::cos(yaw_ * 0.5);

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.linear.z = 0.0;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = wz;

        // Conservative wheel-odometry covariance.
        odom.pose.covariance[0] = 0.05;
        odom.pose.covariance[7] = 0.08;
        odom.pose.covariance[14] = 999999.0;
        odom.pose.covariance[21] = 999999.0;
        odom.pose.covariance[28] = 999999.0;
        odom.pose.covariance[35] = 0.10;

        odom.twist.covariance[0] = 0.03;
        odom.twist.covariance[7] = 0.05;
        odom.twist.covariance[14] = 999999.0;
        odom.twist.covariance[21] = 999999.0;
        odom.twist.covariance[28] = 999999.0;
        odom.twist.covariance[35] = 0.08;

        wheel_odom_pub_->publish(odom);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
