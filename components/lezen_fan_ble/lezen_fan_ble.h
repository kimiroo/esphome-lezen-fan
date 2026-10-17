// lezen_fan_ble.h
// ESPHome external_component: MyFans BLE remote protocol
// (ported from the whitening/CRC algorithm verified against the original
// Android app's btsnoop capture)
//
// Uses raw ESP-IDF BLE GAP APIs directly (esp_ble_gap_config_adv_data_raw /
// esp_ble_gap_start_advertising) so the exact verified byte structure
// (Flags AD 02 01 02 + Manufacturer AD FF F0 FF <18 bytes>) is sent
// unmodified, with no Arduino/NimBLE abstraction layer in between.
//
// Requires: esp32: framework: type: esp-idf, and an `esp32_ble:` component
// declared in the same YAML (it owns the controller/Bluedroid init lifecycle;
// this component just uses the GAP API once BT is already enabled).

#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include "esphome/core/helpers.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"

namespace esphome {
namespace lezen_fan_ble {

static const char *const TAG = "lezen_fan_ble";
static const uint32_t FAN_ADV_HOLD_MS = 200;  // matches the original Java app timing

class FanBLEComponent : public Component {
 public:
  void setup() override {
    this->pref_ = global_preferences->make_preference<uint16_t>(fnv1_hash("lezen_fan_ble_devid"));
    load_or_create_device_id();
    ESP_LOGI(TAG, "ready. device_id=%s", device_id_hex_);
  }

  // depend on esp32_ble having already brought up the BT controller/Bluedroid
  float get_setup_priority() const override { return setup_priority::AFTER_BLUETOOTH; }

  // public API called from YAML lambdas -----------------------------

  void pair() {
    generate_random_device_id();
    send_cmd(14);
  }

  void send_cmd(int cmd_type) {
    uint8_t out[18];
    generate_command(cmd_type, out);
    ESP_LOGI(TAG, "cmd %d (id=%s)", cmd_type, device_id_hex_);
    broadcast_payload(out);
  }

  const char *device_id() const { return device_id_hex_; }

 protected:
  ESPPreferenceObject pref_;
  uint8_t device_nibbles_[4];
  char device_id_hex_[5] = {0, 0, 0, 0, 0};

  // ---------------- whitening / CRC (ported from RF297L.java) ----------------

  static uint8_t invert8(uint8_t v) {
    uint8_t out = 0;
    for (int i = 0; i < 8; i++)
      if (v & (1 << i)) out |= (1 << (7 - i));
    return out;
  }

  static uint16_t invert16(uint16_t v) {
    uint16_t out = 0;
    for (int i = 0; i < 16; i++)
      if (v & (1 << i)) out |= (1 << (15 - i));
    return out;
  }

  static void whitening_init(int index, int reg[7]) {
    reg[0] = 1;
    for (int i = 1; i < 7; i++) reg[i] = (index >> (6 - i)) & 1;
  }

  static int whitening_output(int reg[7]) {
    int i = reg[3], i2 = reg[6];
    reg[3] = reg[2]; reg[2] = reg[1]; reg[1] = reg[0]; reg[0] = i2;
    reg[6] = reg[5]; reg[5] = reg[4]; reg[4] = i ^ i2;
    return i2;
  }

  static void whitening_encode(uint8_t *data, int len, int reg[7]) {
    for (int i = 0; i < len; i++) {
      uint8_t b = data[i];
      int acc = 0;
      for (int bit = 0; bit < 8; bit++)
        acc += (((b >> bit) & 1) ^ whitening_output(reg)) << bit;
      data[i] = acc & 0xFF;
    }
  }

  static uint16_t check_crc16(const uint8_t *addr, int addr_len, const uint8_t *payload, int payload_len) {
    uint16_t c = 0xFFFF;
    for (int i = 0; i < addr_len; i++) {
      c = (c ^ (addr[addr_len - 1 - i] << 8)) & 0xFFFF;
      for (int b = 0; b < 8; b++) c = (c & 0x8000) ? ((c << 1) ^ 4129) & 0xFFFF : (c << 1) & 0xFFFF;
    }
    for (int i = 0; i < payload_len; i++) {
      c = ((invert8(payload[i]) << 8) ^ c) & 0xFFFF;
      for (int b = 0; b < 8; b++) c = (c & 0x8000) ? ((c << 1) ^ 4129) & 0xFFFF : (c << 1) & 0xFFFF;
    }
    return (invert16(c) ^ 0xFFFF) & 0xFFFF;
  }

  static void get_whitening_payload(uint8_t channel, const uint8_t addr[5], const uint8_t payload[8], uint8_t out[18]) {
    int r1[7], r2[7];
    whitening_init(channel, r1);
    whitening_init(63, r2);

    const int n = 5 + 18 + 8 + 2;  // 33
    uint8_t b[33] = {0};
    b[15] = 113; b[16] = 15; b[17] = 85;
    for (int i = 0; i < 5; i++) b[i + 18] = addr[5 - i - 1];
    for (int i = 0; i < 8; i++) b[5 + 18 + i] = payload[i];
    for (int i = 0; i < 5 + 3; i++) b[i + 15] = invert8(b[i + 15]);

    uint16_t crc = check_crc16(addr, 5, payload, 8);
    b[5 + 18 + 8] = crc & 0xFF;
    b[5 + 18 + 8 + 1] = (crc >> 8) & 0xFF;

    whitening_encode(b + 18, n - 18, r2);
    whitening_encode(b, n, r1);

    memcpy(out, b + 15, 18);
  }

  void generate_command(int cmd_type, uint8_t out[18]) {
    uint8_t cmd_byte;
    uint8_t id_bytes[4];

    if (cmd_type == 15) {
      cmd_byte = 0xB5;
      for (int i = 0; i < 4; i++) id_bytes[i] = 0xCC;
    } else {
      cmd_byte = (cmd_type < 10) ? (0xA0 + cmd_type) : (0xB0 + (cmd_type - 10));
      for (int i = 0; i < 4; i++) id_bytes[i] = device_nibbles_[i];
    }

    uint8_t rf[8];
    rf[0] = 0xAA; rf[1] = 0x66; rf[2] = cmd_byte;
    rf[3] = id_bytes[0]; rf[4] = id_bytes[1]; rf[5] = id_bytes[2]; rf[6] = id_bytes[3];

    uint16_t sum = 0;
    for (int i = 0; i < 7; i++) sum += rf[i];
    rf[7] = sum & 0xFF;

    const uint8_t default_address[5] = {0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
    get_whitening_payload(37, default_address, rf, out);
  }

  // ---------------- device id management ----------------
  // stored as a packed uint16_t: 4 nibbles x 4 bits, via ESPHome's own
  // preference API (flash-backed, independent of Arduino Preferences.h)

  static char nibble_to_hex_char(uint8_t n) {
    return (n < 10) ? ('0' + n) : ('a' + (n - 10));
  }

  void nibbles_to_hex() {
    for (int i = 0; i < 4; i++) device_id_hex_[i] = nibble_to_hex_char(device_nibbles_[i]);
    device_id_hex_[4] = '\0';
  }

  void generate_random_device_id() {
    for (int i = 0; i < 4; i++) device_nibbles_[i] = (uint8_t) (random_uint32() & 0x0F);
    nibbles_to_hex();

    uint16_t packed = 0;
    for (int i = 0; i < 4; i++) packed |= ((uint16_t) device_nibbles_[i]) << (i * 4);
    this->pref_.save(&packed);
    ESP_LOGI(TAG, "generated and saved new device id: %s", device_id_hex_);
  }

  void load_or_create_device_id() {
    uint16_t packed;
    if (this->pref_.load(&packed)) {
      for (int i = 0; i < 4; i++) device_nibbles_[i] = (packed >> (i * 4)) & 0x0F;
      nibbles_to_hex();
      ESP_LOGI(TAG, "loaded saved device id: %s", device_id_hex_);
    } else {
      generate_random_device_id();
    }
  }

  // ---------------- BLE advertising (raw ESP-IDF GAP API) ----------------

  void broadcast_payload(const uint8_t payload[18]) {
    uint8_t raw[3 + 4 + 18];
    // Flags AD: len=2, type=0x01, value=0x02 (LE General Discoverable Mode)
    raw[0] = 0x02; raw[1] = 0x01; raw[2] = 0x02;
    // Manufacturer Specific Data AD: len=21, type=0xFF, company id 0xFFF0 (LE)
    raw[3] = 0x15; raw[4] = 0xFF; raw[5] = 0xF0; raw[6] = 0xFF;
    memcpy(raw + 7, payload, 18);

    esp_ble_gap_config_adv_data_raw(raw, sizeof(raw));
    // esp_ble_gap_config_adv_data_raw() is asynchronous; a short settle time
    // avoids racing start_advertising() ahead of it landing.
    delay(30);

    static esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x00A0,
        .adv_int_max = 0x00D2,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    esp_ble_gap_start_advertising(&adv_params);

    delay(FAN_ADV_HOLD_MS);

    esp_ble_gap_stop_advertising();
  }
};

}  // namespace lezen_fan_ble
}  // namespace esphome