// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com
// Description: CRSF frame parser for ExpressLRS receivers

#ifndef EVAROBOT_RC__CRSF_PARSER_HPP_
#define EVAROBOT_RC__CRSF_PARSER_HPP_

#include "evarobot_rc/crsf_protocol.hpp"
#include <array>
#include <cstdint>
#include <functional>

namespace evarobot_rc
{

/**
 * @brief Parsed RC channel data
 */
struct RcChannels
{
  std::array<uint16_t, CRSF_NUM_CHANNELS> channels;
  bool valid;
};

/**
 * @brief Parsed link statistics
 */
struct LinkStats
{
  int rssi_dbm;          // Best RSSI in dBm (negative)
  int link_quality;      // 0-100 %
  int snr_db;            // Signal-to-noise ratio in dB
  int rf_mode;
  int tx_power_mw;
  bool valid;
};

/**
 * @brief CRSF protocol frame parser
 *
 * Processes a stream of bytes from an ELRS receiver and extracts
 * RC channel data and link statistics. Handles framing, CRC
 * validation, and byte-level state machine parsing.
 */
class CrsfParser
{
public:
  using ChannelsCallback = std::function<void(const RcChannels &)>;
  using LinkStatsCallback = std::function<void(const LinkStats &)>;

  CrsfParser();

  /**
   * @brief Process incoming bytes from serial
   * @param data Pointer to byte buffer
   * @param length Number of bytes in buffer
   */
  void processBytes(const uint8_t * data, size_t length);

  /**
   * @brief Set callback for when new RC channels are parsed
   */
  void setChannelsCallback(ChannelsCallback callback);

  /**
   * @brief Set callback for when link statistics are parsed
   */
  void setLinkStatsCallback(LinkStatsCallback callback);

  /**
   * @brief Get count of valid frames parsed
   */
  uint32_t getValidFrameCount() const { return valid_frame_count_; }

  /**
   * @brief Get count of CRC errors
   */
  uint32_t getCrcErrorCount() const { return crc_error_count_; }

private:
  /**
   * @brief Process a single byte through the state machine
   */
  void processByte(uint8_t byte);

  /**
   * @brief Handle a complete, validated CRSF frame
   */
  void handleFrame();

  /**
   * @brief Decode RC channels from packed payload
   */
  RcChannels decodeRcChannels(const uint8_t * payload) const;

  /**
   * @brief Decode link statistics from payload
   */
  LinkStats decodeLinkStatistics(const uint8_t * payload) const;

  /**
   * @brief Calculate CRC8 (DVB-S2 polynomial 0xD5)
   */
  uint8_t crc8DvbS2(const uint8_t * data, size_t length) const;

  // Parser state machine
  enum class State
  {
    WAIT_SYNC,
    WAIT_LENGTH,
    READING_PAYLOAD
  };

  State state_;
  uint8_t frame_buffer_[CRSF_MAX_PACKET_SIZE];
  size_t frame_pos_;
  size_t expected_length_;

  // Callbacks
  ChannelsCallback channels_callback_;
  LinkStatsCallback link_stats_callback_;

  // Statistics
  uint32_t valid_frame_count_;
  uint32_t crc_error_count_;
};

}  // namespace evarobot_rc

#endif  // EVAROBOT_RC__CRSF_PARSER_HPP_
