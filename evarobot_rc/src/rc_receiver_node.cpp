// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com
// Description: ROS2 node for ExpressLRS CRSF receiver on Raspberry Pi 4B UART

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "evarobot_rc/crsf_parser.hpp"
#include "evarobot_rc/serial_port.hpp"
#include "evarobot_rc/channel_mapper.hpp"

#include <memory>
#include <thread>
#include <atomic>

namespace evarobot_rc
{

// ============================================================================
// RC RECEIVER NODE
// ============================================================================

class RcReceiverNode : public rclcpp::Node
{
public:
  explicit RcReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rc_receiver_node", options),
    running_(false)
  {
    // ========================================================================
    // PARAMETERS
    // ========================================================================

    // Serial port configuration
    this->declare_parameter("serial_port", "/dev/ttyAMA0");
    this->declare_parameter("baud_rate", 420000);

    // Channel mapping
    this->declare_parameter("throttle_channel", 1);
    this->declare_parameter("steering_channel", 0);
    this->declare_parameter("arm_channel", 4);
    this->declare_parameter("arm_channel_2", -1);
    this->declare_parameter("arm_threshold", 1500);
    this->declare_parameter("deadzone", 30);
    this->declare_parameter("invert_throttle", false);
    this->declare_parameter("invert_steering", false);

    // Velocity limits
    this->declare_parameter("max_linear_vel", 0.5);
    this->declare_parameter("max_angular_vel", 1.5);

    // Topics
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("raw_channels_topic", "rc/channels");
    this->declare_parameter("link_stats_topic", "rc/link_stats");

    // Timing
    this->declare_parameter("failsafe_timeout", 0.5);  // seconds

    // Get parameters
    serial_port_path_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    failsafe_timeout_ = this->get_parameter("failsafe_timeout").as_double();

    // ========================================================================
    // CHANNEL MAPPER
    // ========================================================================

    ChannelMapConfig map_config;
    map_config.throttle_channel = this->get_parameter("throttle_channel").as_int();
    map_config.steering_channel = this->get_parameter("steering_channel").as_int();
    map_config.arm_channel = this->get_parameter("arm_channel").as_int();
    map_config.arm_channel_2 = this->get_parameter("arm_channel_2").as_int();
    map_config.arm_threshold = this->get_parameter("arm_threshold").as_int();
    map_config.deadzone = this->get_parameter("deadzone").as_int();
    map_config.invert_throttle = this->get_parameter("invert_throttle").as_bool();
    map_config.invert_steering = this->get_parameter("invert_steering").as_bool();
    map_config.max_linear_vel = this->get_parameter("max_linear_vel").as_double();
    map_config.max_angular_vel = this->get_parameter("max_angular_vel").as_double();

    channel_mapper_ = std::make_unique<ChannelMapper>(map_config);

    // ========================================================================
    // PUBLISHERS
    // ========================================================================

    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    std::string raw_channels_topic = this->get_parameter("raw_channels_topic").as_string();
    std::string link_stats_topic = this->get_parameter("link_stats_topic").as_string();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    raw_channels_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      raw_channels_topic, 10);
    link_stats_pub_ = this->create_publisher<std_msgs::msg::String>(link_stats_topic, 10);

    // ========================================================================
    // CRSF PARSER
    // ========================================================================

    parser_ = std::make_unique<CrsfParser>();

    parser_->setChannelsCallback(
      [this](const RcChannels & channels) {
        this->onChannelsReceived(channels);
      });

    parser_->setLinkStatsCallback(
      [this](const LinkStats & stats) {
        this->onLinkStatsReceived(stats);
      });

    // ========================================================================
    // SERIAL PORT & READER THREAD
    // ========================================================================

    serial_ = std::make_unique<SerialPort>();

    if (!serial_->open(serial_port_path_, baud_rate_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to open serial port %s @ %d baud", serial_port_path_.c_str(), baud_rate_);
      RCLCPP_ERROR(this->get_logger(), "Check that:");
      RCLCPP_ERROR(this->get_logger(), "  1. ELRS receiver is wired to GPIO 14/15");
      RCLCPP_ERROR(this->get_logger(), "  2. Bluetooth is disabled (dtoverlay=disable-bt)");
      RCLCPP_ERROR(this->get_logger(), "  3. Serial console is disabled on ttyAMA0");
      RCLCPP_ERROR(this->get_logger(), "  4. User has dialout group permission");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Opened serial port %s @ %d baud", serial_port_path_.c_str(), baud_rate_);

      // Start reader thread
      running_ = true;
      reader_thread_ = std::thread(&RcReceiverNode::readerLoop, this);
    }

    // ========================================================================
    // FAILSAFE TIMER
    // ========================================================================

    last_channels_time_ = this->get_clock()->now();
    failsafe_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RcReceiverNode::failsafeCheck, this));

    RCLCPP_INFO(this->get_logger(), "RC Receiver Node started");
    RCLCPP_INFO(
      this->get_logger(),
      "Throttle: CH%d, Steering: CH%d, Arm: CH%d + CH%d",
      map_config.throttle_channel, map_config.steering_channel,
      map_config.arm_channel, map_config.arm_channel_2);
  }

  ~RcReceiverNode()
  {
    running_ = false;
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    serial_->close();

    RCLCPP_INFO(
      this->get_logger(),
      "Shutdown. Valid frames: %u, CRC errors: %u",
      parser_->getValidFrameCount(), parser_->getCrcErrorCount());
  }

private:
  // ========================================================================
  // SERIAL READER THREAD
  // ========================================================================

  void readerLoop()
  {
    uint8_t buffer[256];

    while (running_) {
      int bytes_read = serial_->read(buffer, sizeof(buffer));

      if (bytes_read > 0) {
        parser_->processBytes(buffer, static_cast<size_t>(bytes_read));
      } else if (bytes_read == 0) {
        // No data, sleep briefly to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::microseconds(500));
      } else {
        // Read error
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Serial read error on %s", serial_port_path_.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  // ========================================================================
  // CALLBACKS
  // ========================================================================

  void onChannelsReceived(const RcChannels & channels)
  {
    if (!channels.valid) {
      return;
    }

    last_channels_time_ = this->get_clock()->now();
    in_failsafe_ = false;

    // Publish raw channels
    auto raw_msg = std_msgs::msg::Int32MultiArray();
    raw_msg.data.resize(CRSF_NUM_CHANNELS);
    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i) {
      raw_msg.data[i] = static_cast<int32_t>(channels.channels[i]);
    }
    raw_channels_pub_->publish(raw_msg);

    // Map to cmd_vel and publish
    auto twist = channel_mapper_->mapToTwist(channels.channels);
    cmd_vel_pub_->publish(twist);
  }

  void onLinkStatsReceived(const LinkStats & stats)
  {
    if (!stats.valid) {
      return;
    }

    auto msg = std_msgs::msg::String();
    msg.data = "rssi=" + std::to_string(stats.rssi_dbm) +
      " lq=" + std::to_string(stats.link_quality) +
      " snr=" + std::to_string(stats.snr_db) +
      " txpwr=" + std::to_string(stats.tx_power_mw) + "mW";
    link_stats_pub_->publish(msg);
  }

  // ========================================================================
  // FAILSAFE
  // ========================================================================

  void failsafeCheck()
  {
    double elapsed = (this->get_clock()->now() - last_channels_time_).seconds();

    if (elapsed > failsafe_timeout_ && !in_failsafe_) {
      in_failsafe_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "RC failsafe triggered - no CRSF data for %.1f seconds", elapsed);

      // Publish zero velocity
      auto twist = geometry_msgs::msg::Twist();
      cmd_vel_pub_->publish(twist);
    }
  }

  // Members
  std::unique_ptr<SerialPort> serial_;
  std::unique_ptr<CrsfParser> parser_;
  std::unique_ptr<ChannelMapper> channel_mapper_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr raw_channels_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr link_stats_pub_;

  // Reader thread
  std::thread reader_thread_;
  std::atomic<bool> running_;

  // Failsafe
  rclcpp::TimerBase::SharedPtr failsafe_timer_;
  rclcpp::Time last_channels_time_;
  double failsafe_timeout_;
  bool in_failsafe_ = false;

  // Config
  std::string serial_port_path_;
  int baud_rate_;
};

}  // namespace evarobot_rc

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<evarobot_rc::RcReceiverNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
