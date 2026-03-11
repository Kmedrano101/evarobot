// Author: Kevin Medrano Ayala
// Contact: kevin.ejem18@gmail.com
// Description: CRSF frame parser implementation

#include "evarobot_rc/crsf_parser.hpp"
#include <cstring>

namespace evarobot_rc
{

// ============================================================================
// CRC8 DVB-S2 LOOKUP TABLE (polynomial 0xD5)
// ============================================================================

static const uint8_t crc8_dvb_s2_table[256] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
  0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
  0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
  0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
  0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
  0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
  0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
  0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
  0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
  0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
  0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
  0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
  0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
  0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
  0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
  0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
  0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
};

// ============================================================================
// CONSTRUCTOR
// ============================================================================

CrsfParser::CrsfParser()
: state_(State::WAIT_SYNC),
  frame_pos_(0),
  expected_length_(0),
  valid_frame_count_(0),
  crc_error_count_(0)
{
  std::memset(frame_buffer_, 0, sizeof(frame_buffer_));
}

// ============================================================================
// PUBLIC METHODS
// ============================================================================

void CrsfParser::processBytes(const uint8_t * data, size_t length)
{
  for (size_t i = 0; i < length; ++i) {
    processByte(data[i]);
  }
}

void CrsfParser::setChannelsCallback(ChannelsCallback callback)
{
  channels_callback_ = std::move(callback);
}

void CrsfParser::setLinkStatsCallback(LinkStatsCallback callback)
{
  link_stats_callback_ = std::move(callback);
}

// ============================================================================
// STATE MACHINE
// ============================================================================

void CrsfParser::processByte(uint8_t byte)
{
  switch (state_) {
    case State::WAIT_SYNC:
      if (byte == CRSF_SYNC_BYTE) {
        frame_buffer_[0] = byte;
        frame_pos_ = 1;
        state_ = State::WAIT_LENGTH;
      }
      break;

    case State::WAIT_LENGTH:
      // Length byte: type (1) + payload (N) + crc (1)
      // Valid range: 2 to CRSF_MAX_FRAME_SIZE
      if (byte >= 2 && byte <= CRSF_MAX_FRAME_SIZE) {
        frame_buffer_[1] = byte;
        expected_length_ = byte;
        frame_pos_ = 2;
        state_ = State::READING_PAYLOAD;
      } else {
        // Invalid length, reset
        state_ = State::WAIT_SYNC;
      }
      break;

    case State::READING_PAYLOAD:
      frame_buffer_[frame_pos_++] = byte;

      // Check if we've received the full frame (sync + length + payload)
      if (frame_pos_ >= expected_length_ + 2) {
        handleFrame();
        state_ = State::WAIT_SYNC;
      }
      break;
  }
}

void CrsfParser::handleFrame()
{
  // Frame layout: [sync][length][type][payload...][crc]
  // CRC is calculated over type + payload (excludes sync, length, and crc itself)
  uint8_t frame_type = frame_buffer_[2];
  size_t crc_data_length = expected_length_ - 1;  // type + payload (no crc)
  uint8_t received_crc = frame_buffer_[frame_pos_ - 1];
  uint8_t calculated_crc = crc8DvbS2(&frame_buffer_[2], crc_data_length);

  if (received_crc != calculated_crc) {
    crc_error_count_++;
    return;
  }

  valid_frame_count_++;

  // Dispatch based on frame type
  auto type = static_cast<CrsfFrameType>(frame_type);

  switch (type) {
    case CrsfFrameType::RC_CHANNELS_PACKED:
      if (channels_callback_) {
        RcChannels channels = decodeRcChannels(&frame_buffer_[3]);
        channels_callback_(channels);
      }
      break;

    case CrsfFrameType::LINK_STATISTICS:
      if (link_stats_callback_) {
        LinkStats stats = decodeLinkStatistics(&frame_buffer_[3]);
        link_stats_callback_(stats);
      }
      break;

    default:
      // Ignore other frame types
      break;
  }
}

// ============================================================================
// FRAME DECODERS
// ============================================================================

RcChannels CrsfParser::decodeRcChannels(const uint8_t * payload) const
{
  RcChannels result;
  result.valid = true;

  // RC channels are packed as 16 x 11-bit values = 22 bytes
  const auto * packed = reinterpret_cast<const CrsfRcChannelsPacked *>(payload);

  result.channels[0] = packed->ch0;
  result.channels[1] = packed->ch1;
  result.channels[2] = packed->ch2;
  result.channels[3] = packed->ch3;
  result.channels[4] = packed->ch4;
  result.channels[5] = packed->ch5;
  result.channels[6] = packed->ch6;
  result.channels[7] = packed->ch7;
  result.channels[8] = packed->ch8;
  result.channels[9] = packed->ch9;
  result.channels[10] = packed->ch10;
  result.channels[11] = packed->ch11;
  result.channels[12] = packed->ch12;
  result.channels[13] = packed->ch13;
  result.channels[14] = packed->ch14;
  result.channels[15] = packed->ch15;

  return result;
}

LinkStats CrsfParser::decodeLinkStatistics(const uint8_t * payload) const
{
  LinkStats result;
  result.valid = true;

  const auto * stats = reinterpret_cast<const CrsfLinkStatistics *>(payload);

  // Use the better of the two antenna RSSI values
  int rssi1 = -static_cast<int>(stats->uplink_rssi_ant1);
  int rssi2 = -static_cast<int>(stats->uplink_rssi_ant2);
  result.rssi_dbm = (rssi1 > rssi2) ? rssi1 : rssi2;

  result.link_quality = static_cast<int>(stats->uplink_link_quality);
  result.snr_db = static_cast<int>(stats->uplink_snr);
  result.rf_mode = static_cast<int>(stats->rf_mode);

  // Decode TX power from enum to mW
  static const int tx_power_lut[] = {0, 10, 25, 100, 500, 1000, 2000, 250, 50};
  int power_idx = static_cast<int>(stats->uplink_tx_power);
  if (power_idx >= 0 && power_idx < static_cast<int>(sizeof(tx_power_lut) / sizeof(int))) {
    result.tx_power_mw = tx_power_lut[power_idx];
  } else {
    result.tx_power_mw = 0;
  }

  return result;
}

// ============================================================================
// CRC
// ============================================================================

uint8_t CrsfParser::crc8DvbS2(const uint8_t * data, size_t length) const
{
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc = crc8_dvb_s2_table[crc ^ data[i]];
  }
  return crc;
}

}  // namespace evarobot_rc
