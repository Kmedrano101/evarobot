// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com
// Description: ROS2 node for controlling EvaRobot motors via serial (Arduino RP2040)

#ifndef EVAROBOT_HARDWARE__MOTOR_CONTROLLER_HPP_
#define EVAROBOT_HARDWARE__MOTOR_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <string>

namespace evarobot_hardware
{

/**
 * @brief Motor controller node for EvaRobot differential drive
 *
 * Subscribes to cmd_vel (Twist) messages and translates them to
 * JSON commands for the Arduino RP2040 motor controller via serial.
 */
class MotorController : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  explicit MotorController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor - stops motors and closes serial
   */
  ~MotorController();

private:
  /**
   * @brief Callback for cmd_vel topic
   * @param msg Twist message with linear and angular velocities
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Convert velocity (m/s) to PWM duty cycle
   * @param velocity Linear velocity in m/s
   * @return PWM duty cycle (min_speed_ to max_speed_)
   */
  int velocityToSpeed(double velocity) const;

  /**
   * @brief Send motor command to Arduino via serial
   * @param status Motor status command (forward, backward, turnleft, turnright, stop)
   * @param speed_left Left motor PWM duty cycle
   * @param speed_right Right motor PWM duty cycle
   */
  void sendMotorCommand(
    const std::string & status,
    int speed_left,
    int speed_right);

  /**
   * @brief Send stop command to motors
   */
  void sendStopCommand();

  /**
   * @brief Initialize serial connection
   * @return true if successful
   */
  bool initializeSerial();

  // ROS2 subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Serial connection handle (pimpl idiom to avoid exposing serial library)
  class SerialImpl;
  std::unique_ptr<SerialImpl> serial_impl_;

  // Parameters
  std::string serial_port_;
  int baud_rate_;
  int min_speed_;
  int max_speed_;
  double wheel_base_;
  double max_linear_velocity_;
  double cmd_vel_timeout_;

  // Watchdog timer for cmd_vel timeout
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_time_;
};

}  // namespace evarobot_hardware

#endif  // EVAROBOT_HARDWARE__MOTOR_CONTROLLER_HPP_
