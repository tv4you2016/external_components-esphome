#pragma once

// ============================================================
// bt_a2dp_sink.h — Native ESP-IDF Bluetooth A2DP Sink
// ============================================================
//
// Routes Bluetooth Classic A2DP audio through the ESPHome
// speaker pipeline. No external libraries required — uses
// Espressif's Bluedroid stack already compiled into the
// ESP-IDF used by ESPHome.
//
// Audio routing (BT connected):
//   A2DP SBC decode (44100 Hz 16-bit stereo)
//     → media_resampling_speaker  (resample 44100 → 48000 Hz)
//       → media_mixer_input
//         → mixer_speaker_id
//           → i2s_speaker_id      (I2S @ 48000 Hz)
//             → TAS5825M DAC      (DSP / 15-band EQ preserved)
//
// Cross-task safety:
//   The ESP-IDF A2DP callback runs in a Bluetooth FreeRTOS
//   task. ESPHome runs in the main task. All ESPHome API
//   calls (media_player, switch, binary_sensor) are deferred
//   via a flag checked in loop() to avoid race conditions.
//   The PCM data callback writes directly to the speaker's
//   DMA ring buffer which is thread-safe by design.
// ============================================================

#include "esphome/core/component.h"
#include "esphome/components/speaker/speaker.h"
#include "esphome/components/media_player/media_player.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"

// Native ESP-IDF Bluetooth headers.
// These ship with the ESP-IDF already bundled into ESPHome.
// No external library or esphome: libraries: entry is needed.
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

namespace esphome {
namespace bt_a2dp_sink {

// ── Static pointers (set once in setup, read-only after that) ──────────────
static speaker::Speaker *s_speaker = nullptr;
static media_player::MediaPlayer *s_media_player = nullptr;
static binary_sensor::BinarySensor *s_conn_sensor = nullptr;
static switch_::Switch *s_dac_switch = nullptr;

// ── Deferred action flag (written by BT task, read by main loop) ───────────
// Using a simple enum guarded by atomic-width type. The ESP32's Xtensa
// architecture guarantees 32-bit aligned writes are atomic.
enum class PendingAction : uint32_t { NONE = 0, CONNECTED = 1, DISCONNECTED = 2 };
static volatile PendingAction s_pending_action = PendingAction::NONE;

// ── A2DP PCM data callback ─────────────────────────────────────────────────
// Called from the Bluetooth FreeRTOS task with raw SBC-decoded PCM.
// Standard A2DP SBC output: 44100 Hz, 16-bit, stereo.
//
// We write directly to the resampler speaker (media_resampling_speaker).
// Its internal ring buffer is thread-safe. ticks_to_wait=0 is non-blocking:
// if the DMA buffer is momentarily full, this packet is dropped rather than
// stalling the Bluetooth stack (which would cause connection instability).
static void a2dp_data_callback(const uint8_t *data, uint32_t len) {
  if (s_speaker != nullptr) {
    s_speaker->play(data, len, 0);
  }
}

// ── A2DP connection/audio state callback ───────────────────────────────────
// Called from the Bluetooth FreeRTOS task. Only sets the deferred flag —
// all ESPHome API calls happen in loop() on the main task.
static void a2dp_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
  switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
      const auto state = param->conn_stat.state;
      if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        ESP_LOGI("bt_a2dp", "Bluetooth device connected");
        s_pending_action = PendingAction::CONNECTED;
      } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        ESP_LOGI("bt_a2dp", "Bluetooth device disconnected");
        s_pending_action = PendingAction::DISCONNECTED;
      }
      break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
      // Stream start/stop events available here if needed in future
      break;
    default:
      break;
  }
}

// ── GAP callback ───────────────────────────────────────────────────────────
// Required registration; no action needed for basic A2DP sink.
static void gap_cb(esp_bt_gap_cb_event_t event,
                   esp_bt_gap_cb_param_t *param) {
  (void)event;
  (void)param;
}

// ── Component class ────────────────────────────────────────────────────────
class BTA2DPSinkComponent : public Component {
 public:
  // ── Setters called from to_code() generated code ─────────────────────────
  void set_speaker(speaker::Speaker *spk) { s_speaker = spk; }
  void set_media_player(media_player::MediaPlayer *mp) { s_media_player = mp; }
  void set_connected_sensor(binary_sensor::BinarySensor *s) { s_conn_sensor = s; }
  void set_dac_switch(switch_::Switch *sw) { s_dac_switch = sw; }
  void set_device_name(const std::string &name) { device_name_ = name; }

  // ── setup() ───────────────────────────────────────────────────────────────
  // Initialises the ESP-IDF Bluedroid stack and starts the A2DP sink.
  // Runs after i2s_audio, speaker, and media_player are all ready.
  void setup() override {
    ESP_LOGI("bt_a2dp", "Initialising native ESP-IDF A2DP sink...");

    // Release BLE controller memory — only Classic BT (BR/EDR) is needed.
    // This reclaims ~70 KB of internal SRAM for the A2DP stack.
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // Initialise and enable the Bluetooth controller in Classic BT mode.
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    // Initialise and enable the Bluedroid host stack.
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Register callbacks.
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_a2d_register_callback(a2dp_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(a2dp_data_callback));

    // Initialise the A2DP sink profile.
    ESP_ERROR_CHECK(esp_a2d_sink_init());

    // Set the device name that appears in the phone's Bluetooth list.
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(device_name_.c_str()));

    // Make the device connectable and discoverable.
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(
        ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));

    // Publish initial disconnected state.
    if (s_conn_sensor != nullptr) {
      s_conn_sensor->publish_state(false);
    }

    ESP_LOGI("bt_a2dp", "A2DP sink ready as '%s'", device_name_.c_str());
  }

  // ── loop() ────────────────────────────────────────────────────────────────
  // Processes deferred connection state changes on the main ESPHome task.
  void loop() override {
    const PendingAction action = s_pending_action;
    if (action == PendingAction::NONE) return;
    s_pending_action = PendingAction::NONE;

    if (action == PendingAction::CONNECTED) {
      // 1. Stop the Sendspin/media-player pipeline so it releases the audio
      //    pipeline. The speaker stack (resampler/mixer/I2S) stays alive
      //    because all speakers are configured with timeout: never.
      if (s_media_player != nullptr) {
        auto call = s_media_player->make_call();
        call.set_command_stop();
        call.perform();
      }
      // 2. Ensure the TAS58xx DAC is powered on. The sendspin-addon-dac-enable
      //    package turns the DAC off on player idle; we need to re-enable it
      //    for Bluetooth playback since it bypasses the media player's on_play.
      if (s_dac_switch != nullptr && !s_dac_switch->state) {
        s_dac_switch->turn_on();
      }
      // 3. Start the speaker so the I2S peripheral is clocked and the DMA
      //    buffers are active and ready to accept incoming PCM from the
      //    A2DP data callback.
      if (s_speaker != nullptr) {
        s_speaker->start();
      }
      // 4. Publish connected state to HA.
      if (s_conn_sensor != nullptr) {
        s_conn_sensor->publish_state(true);
      }
      ESP_LOGI("bt_a2dp", "BT source active — Sendspin paused");

    } else if (action == PendingAction::DISCONNECTED) {
      // 1. Stop the speaker to flush the DMA buffers and prevent noise.
      //    The DAC switch is intentionally left ON — sendspin-addon-dac-enable
      //    will handle turning it off when the media player next goes idle,
      //    avoiding a double-toggle race condition.
      if (s_speaker != nullptr) {
        s_speaker->stop();
      }
      // 2. Publish disconnected state to HA.
      if (s_conn_sensor != nullptr) {
        s_conn_sensor->publish_state(false);
      }
      ESP_LOGI("bt_a2dp", "BT source released — Sendspin will resume on next push");
    }
  }

  // Run after i2s_audio, speaker, and media_player are all set up.
  float get_setup_priority() const override { return setup_priority::LATE; }

 private:
  std::string device_name_{"Audio Brick"};
};

}  // namespace bt_a2dp_sink
}  // namespace esphome
