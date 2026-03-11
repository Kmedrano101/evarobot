// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com
// Description: Maps CRSF RC channels to velocity commands

#ifndef EVAROBOT_RC__CHANNEL_MAPPER_HPP_
#define EVAROBOT_RC__CHANNEL_MAPPER_HPP_

#include "evarobot_rc/crsf_protocol.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <array>
#include <chrono>
#include <cstdint>

namespace evarobot_rc
{

/**
 * @brief Configuration for channel-to-velocity mapping
 */
struct ChannelMapConfig
{
  int throttle_channel;     // Channel index for linear velocity
  int steering_channel;     // Channel index for angular velocity
  int arm_channel;          // Channel index for arm switch (-1 to disable)
  int arm_channel_2;        // Second arm channel (-1 to disable, both must be armed)
  int arm_threshold;        // CRSF value above which system is armed
  double max_linear_vel;    // Maximum linear velocity (m/s)
  double max_angular_vel;   // Maximum angular velocity (rad/s)
  int deadzone;             // Center deadzone in CRSF units
  bool invert_throttle;     // Invert throttle direction
  bool invert_steering;     // Invert steering direction
};

/**
 * @brief Maps RC channel values to geometry_msgs::msg::Twist
 *
 * Converts raw 11-bit CRSF channel values (172-1811) to normalized
 * velocity commands, applying deadzone filtering, scaling, and
 * arm switch logic.
 */
class ChannelMapper
{
public:
  /**
   * @brief Constructor
   * @param config Channel mapping configuration
   */
  explicit ChannelMapper(const ChannelMapConfig & config);

  /**
   * @brief Convert RC channels to Twist velocity command
   * @param channels Array of 16 raw CRSF channel values
   * @return Twist message with linear.x and angular.z
   */
  geometry_msgs::msg::Twist mapToTwist(
    const std::array<uint16_t, CRSF_NUM_CHANNELS> & channels) const;

  /**
   * @brief Check if system is armed based on arm channel
   * @param channels Array of 16 raw CRSF channel values
   * @return true if armed (or if arm channel is disabled)
   */
  bool isArmed(
    const std::array<uint16_t, CRSF_NUM_CHANNELS> & channels) const;

  /**
   * @brief Update the configuration
   */
  void setConfig(const ChannelMapConfig & config);

private:
  /**
   * @brief Normalize CRSF channel value to -1.0 to 1.0 with deadzone
   * @param value Raw CRSF channel value (172-1811)
   * @return Normalized value in range [-1.0, 1.0]
   */
  double normalizeChannel(uint16_t value) const;

  ChannelMapConfig config_;
  mutable bool armed_latch_ = false;
  mutable bool both_were_high_ = false;
  mutable std::chrono::steady_clock::time_point last_arm_toggle_time_{};
};

}  // namespace evarobot_rc

#endif  // EVAROBOT_RC__CHANNEL_MAPPER_HPP_
