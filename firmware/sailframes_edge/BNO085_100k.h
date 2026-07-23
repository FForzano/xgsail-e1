// BNO085_100k.h - Custom BNO085 driver for GY-BNO08X modules
// Uses 100kHz I2C with reset sequence (works around 400kHz failures)

#ifndef BNO085_100K_H
#define BNO085_100K_H

#include <Wire.h>

class BNO085_100k {
public:
  // Configuration
  static const uint8_t DEFAULT_ADDR = 0x4B;

  // SHTP channels
  static const uint8_t CHANNEL_COMMAND = 0;
  static const uint8_t CHANNEL_EXECUTABLE = 1;
  static const uint8_t CHANNEL_SENSOR_HUB = 2;
  static const uint8_t CHANNEL_INPUT_SENSOR = 3;

  // Report IDs
  static const uint8_t REPORT_GAME_ROTATION_VECTOR = 0x08;
  static const uint8_t REPORT_SET_FEATURE_COMMAND = 0xFD;

  BNO085_100k() : _addr(DEFAULT_ADDR), _wire(&Wire) {}

  bool begin(TwoWire* wire = &Wire, uint8_t addr = DEFAULT_ADDR) {
    _wire = wire;
    _addr = addr;

    // Verify device present
    _wire->beginTransmission(_addr);
    if (_wire->endTransmission() != 0) {
      return false;
    }

    // Reset the device
    if (!reset()) {
      return false;
    }

    delay(100);
    return true;
  }

  bool reset() {
    // Reset command on Executable channel
    uint8_t resetCmd[] = {1};
    if (!sendPacket(CHANNEL_EXECUTABLE, resetCmd, 1)) {
      return false;
    }

    delay(500);

    // Drain pending packets
    for (int i = 0; i < 20; i++) {
      delay(10);
      if (!receivePacket()) break;
    }

    return true;
  }

  bool enableGameRotationVector(uint32_t intervalUs = 50000) {
    uint8_t cmd[17] = {0};
    cmd[0] = REPORT_SET_FEATURE_COMMAND;
    cmd[1] = REPORT_GAME_ROTATION_VECTOR;
    cmd[2] = 0;  // Feature flags
    cmd[3] = 0;  // Change sensitivity LSB
    cmd[4] = 0;  // Change sensitivity MSB
    cmd[5] = (intervalUs >> 0) & 0xFF;
    cmd[6] = (intervalUs >> 8) & 0xFF;
    cmd[7] = (intervalUs >> 16) & 0xFF;
    cmd[8] = (intervalUs >> 24) & 0xFF;

    if (!sendPacket(CHANNEL_SENSOR_HUB, cmd, 17)) {
      return false;
    }

    delay(50);

    // Read confirmation
    for (int i = 0; i < 5; i++) {
      if (receivePacket()) {
        if (_channel == CHANNEL_SENSOR_HUB && _data[0] == 0xFC) {
          return true;
        }
      }
      delay(10);
    }

    return true;  // Proceed even without confirmation
  }

  bool update() {
    if (!receivePacket()) return false;
    if (_channel != CHANNEL_INPUT_SENSOR) return false;
    if (_data[0] != REPORT_GAME_ROTATION_VECTOR) return false;

    // Parse quaternion (Q14 fixed point)
    int16_t rawI = (int16_t)((uint16_t)_data[5] | ((uint16_t)_data[6] << 8));
    int16_t rawJ = (int16_t)((uint16_t)_data[7] | ((uint16_t)_data[8] << 8));
    int16_t rawK = (int16_t)((uint16_t)_data[9] | ((uint16_t)_data[10] << 8));
    int16_t rawReal = (int16_t)((uint16_t)_data[11] | ((uint16_t)_data[12] << 8));

    float scale = 1.0f / (1 << 14);
    quatI = rawI * scale;
    quatJ = rawJ * scale;
    quatK = rawK * scale;
    quatReal = rawReal * scale;

    // Convert to Euler angles (degrees)
    float sinr_cosp = 2.0f * (quatReal * quatI + quatJ * quatK);
    float cosr_cosp = 1.0f - 2.0f * (quatI * quatI + quatJ * quatJ);
    roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;

    float sinp = 2.0f * (quatReal * quatJ - quatK * quatI);
    if (abs(sinp) >= 1)
      pitch = copysign(90.0f, sinp);
    else
      pitch = asin(sinp) * 180.0f / PI;

    float siny_cosp = 2.0f * (quatReal * quatK + quatI * quatJ);
    float cosy_cosp = 1.0f - 2.0f * (quatJ * quatJ + quatK * quatK);
    yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;

    return true;
  }

  // Public data - updated by update()
  float quatI = 0, quatJ = 0, quatK = 0, quatReal = 1;
  float roll = 0;   // Heel angle (degrees)
  float pitch = 0;  // Pitch angle (degrees)
  float yaw = 0;    // Heading (degrees, unreliable without magnetometer)

private:
  uint8_t _addr;
  TwoWire* _wire;
  uint8_t _data[128];
  uint16_t _length;
  uint8_t _channel;
  uint8_t _seq[6] = {0};

  bool sendPacket(uint8_t channel, uint8_t* data, uint16_t length) {
    uint16_t packetLength = length + 4;

    _wire->beginTransmission(_addr);
    _wire->write(packetLength & 0xFF);
    _wire->write((packetLength >> 8) & 0x7F);
    _wire->write(channel);
    _wire->write(_seq[channel]++);

    for (uint16_t i = 0; i < length; i++) {
      _wire->write(data[i]);
    }

    return (_wire->endTransmission() == 0);
  }

  bool receivePacket() {
    _wire->requestFrom(_addr, (uint8_t)4);
    if (_wire->available() < 4) return false;

    uint8_t hdr[4];
    for (int i = 0; i < 4; i++) {
      hdr[i] = _wire->read();
    }

    _length = (uint16_t)hdr[0] | ((uint16_t)(hdr[1] & 0x7F) << 8);
    _channel = hdr[2];

    if (_length == 0 || _length > 128) return false;

    uint16_t dataLength = _length - 4;
    if (dataLength > 0) {
      _wire->requestFrom(_addr, (uint8_t)min(dataLength, (uint16_t)124));
      uint16_t i = 0;
      while (_wire->available() && i < dataLength) {
        _data[i++] = _wire->read();
      }
    }

    return true;
  }
};

#endif // BNO085_100K_H
