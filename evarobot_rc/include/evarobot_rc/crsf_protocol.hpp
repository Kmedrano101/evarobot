// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com
// Description: CRSF (Crossfire) protocol definitions for ExpressLRS receivers
// Reference: https://github.com/crsf-wg/crsf/wiki

#ifndef EVAROBOT_RC__CRSF_PROTOCOL_HPP_
#define EVAROBOT_RC__CRSF_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

namespace evarobot_rc
{

// ============================================================================
// CRSF PROTOCOL CONSTANTS
// ============================================================================

// Sync byte for CRSF frames from receiver
constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;

// Maximum frame size (excluding sync and length bytes)
constexpr size_t CRSF_MAX_FRAME_SIZE = 64;

// Total buffer size including sync + length
constexpr size_t CRSF_MAX_PACKET_SIZE = CRSF_MAX_FRAME_SIZE + 2;

// Number of RC channels in a CRSF RC channels packed frame
constexpr int CRSF_NUM_CHANNELS = 16;

// RC channel value range (11-bit, 0-1984)
constexpr int CRSF_CHANNEL_VALUE_MIN = 172;
constexpr int CRSF_CHANNEL_VALUE_MID = 992;
constexpr int CRSF_CHANNEL_VALUE_MAX = 1811;

// ============================================================================
// CRSF FRAME TYPES
// ============================================================================

enum class CrsfFrameType : uint8_t
{
  GPS = 0x02,
  VARIO = 0x07,
  BATTERY_SENSOR = 0x08,
  BARO_ALTITUDE = 0x09,
  HEARTBEAT = 0x0B,
  LINK_STATISTICS = 0x14,
  RC_CHANNELS_PACKED = 0x16,
  SUBSET_RC_CHANNELS_PACKED = 0x17,
  LINK_RX_ID = 0x1C,
  LINK_TX_ID = 0x1D,
  ATTITUDE = 0x1E,
  FLIGHT_MODE = 0x21,
  DEVICE_PING = 0x28,
  DEVICE_INFO = 0x29,
  PARAMETER_SETTINGS_ENTRY = 0x2B,
  PARAMETER_READ = 0x2C,
  PARAMETER_WRITE = 0x2D,
  ELRS_STATUS = 0x2E,
  COMMAND = 0x32,
  RADIO_ID = 0x3A,
};

// ============================================================================
// CRSF FRAME STRUCTURES
// ============================================================================

// Raw CRSF frame header
struct CrsfFrameHeader
{
  uint8_t sync;
  uint8_t length;    // length of type + payload + crc
  uint8_t type;
} __attribute__((packed));

// RC channels packed payload (11 bits per channel, 16 channels = 22 bytes)
struct CrsfRcChannelsPacked
{
  unsigned ch0 : 11;
  unsigned ch1 : 11;
  unsigned ch2 : 11;
  unsigned ch3 : 11;
  unsigned ch4 : 11;
  unsigned ch5 : 11;
  unsigned ch6 : 11;
  unsigned ch7 : 11;
  unsigned ch8 : 11;
  unsigned ch9 : 11;
  unsigned ch10 : 11;
  unsigned ch11 : 11;
  unsigned ch12 : 11;
  unsigned ch13 : 11;
  unsigned ch14 : 11;
  unsigned ch15 : 11;
} __attribute__((packed));

// Link statistics payload
struct CrsfLinkStatistics
{
  uint8_t uplink_rssi_ant1;     // dBm * -1
  uint8_t uplink_rssi_ant2;     // dBm * -1
  uint8_t uplink_link_quality;  // 0-100 %
  int8_t uplink_snr;            // dB
  uint8_t active_antenna;
  uint8_t rf_mode;
  uint8_t uplink_tx_power;      // mW (enum)
  uint8_t downlink_rssi;        // dBm * -1
  uint8_t downlink_link_quality;  // 0-100 %
  int8_t downlink_snr;           // dB
} __attribute__((packed));

}  // namespace evarobot_rc

#endif  // EVAROBOT_RC__CRSF_PROTOCOL_HPP_
