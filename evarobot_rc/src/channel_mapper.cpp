// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com
// Description: RC channel to velocity command mapper implementation

#include "evarobot_rc/channel_mapper.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace evarobot_rc
{

ChannelMapper::ChannelMapper(const ChannelMapConfig & config)
: config_(config)
{
}

geometry_msgs::msg::Twist ChannelMapper::mapToTwist(
  const std::array<uint16_t, CRSF_NUM_CHANNELS> & channels) const
{
  geometry_msgs::msg::Twist twist;

  // Check arm switch
  if (!isArmed(channels)) {
    // Return zero twist when disarmed
    return twist;
  }

  // Validate channel indices
  if (config_.throttle_channel < 0 || config_.throttle_channel >= CRSF_NUM_CHANNELS ||
    config_.steering_channel < 0 || config_.steering_channel >= CRSF_NUM_CHANNELS)
  {
    return twist;
  }

  // Normalize channels to -1.0 to 1.0
  double throttle = normalizeChannel(channels[config_.throttle_channel]);
  double steering = normalizeChannel(channels[config_.steering_channel]);

  // Apply inversion
  if (config_.invert_throttle) {
    throttle = -throttle;
  }
  if (config_.invert_steering) {
    steering = -steering;
  }

  // Scale to velocity
  twist.linear.x = throttle * config_.max_linear_vel;
  twist.angular.z = steering * config_.max_angular_vel;

  return twist;
}

bool ChannelMapper::isArmed(
  const std::array<uint16_t, CRSF_NUM_CHANNELS> & channels) const
{
  // If both arm channels disabled, always armed
  bool has_arm1 = (config_.arm_channel >= 0 && config_.arm_channel < CRSF_NUM_CHANNELS);
  bool has_arm2 = (config_.arm_channel_2 >= 0 && config_.arm_channel_2 < CRSF_NUM_CHANNELS);

  if (!has_arm1 && !has_arm2) {
    return true;
  }

  // Single arm channel: direct threshold check
  if (!has_arm2) {
    return channels[config_.arm_channel] > static_cast<uint16_t>(config_.arm_threshold);
  }

  uint16_t thresh = static_cast<uint16_t>(config_.arm_threshold);
  bool sw1_high = has_arm1 && channels[config_.arm_channel] > thresh;
  bool sw2_high = has_arm2 && channels[config_.arm_channel_2] > thresh;
  bool both_high = sw1_high && sw2_high;

  // Toggle latch with debounce: press both buttons to toggle arm state
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_arm_toggle_time_).count();

  // On rising edge (both go high) with 500ms debounce, flip armed state
  if (both_high && !both_were_high_ && elapsed > 500) {
    armed_latch_ = !armed_latch_;
    last_arm_toggle_time_ = now;
  }
  both_were_high_ = both_high;

  return armed_latch_;
}

void ChannelMapper::setConfig(const ChannelMapConfig & config)
{
  config_ = config;
}

double ChannelMapper::normalizeChannel(uint16_t value) const
{
  // Clamp to valid CRSF range
  int clamped = std::clamp(
    static_cast<int>(value),
    CRSF_CHANNEL_VALUE_MIN,
    CRSF_CHANNEL_VALUE_MAX);

  // Center around midpoint
  int centered = clamped - CRSF_CHANNEL_VALUE_MID;

  // Apply deadzone
  if (std::abs(centered) < config_.deadzone) {
    return 0.0;
  }

  // Remove deadzone from range for smooth response
  double sign = (centered > 0) ? 1.0 : -1.0;
  double magnitude = std::abs(centered) - config_.deadzone;
  double max_range = (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID) - config_.deadzone;

  if (max_range <= 0.0) {
    return 0.0;
  }

  return sign * std::clamp(magnitude / max_range, 0.0, 1.0);
}

}  // namespace evarobot_rc
