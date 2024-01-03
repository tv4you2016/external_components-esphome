esphome:
  name: isound
  friendly_name: iSound

esp32:
  board: esp32dev
  framework:
    type: arduino

external_components:
  - source:
      type: git
      url: https://github.com/tv4you2016/iSound-esphome
    components: [ i2s_isound ]


# Enable logging
logger:
  level: VERBOSE
  logs:
    isound.component: VERBOSE

# Enable Home Assistant API
api:
  encryption:
    key: "nRO7yR6ToxNrcf1JHcNIk4nFEPHN17I/q12KuT8Ph/k="

ota:
  password: "be71c0cea304b6f32b15420941ecaba9"

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

  # Enable fallback hotspot (captive portal) in case wifi connection fails
  ap:
    ssid: "Isound Fallback Hotspot"
    password: "XFPDNn1Q0GFL"

web_server:
  port: 80
  
captive_portal:

sensor:
  - platform: wifi_signal # Reports the WiFi signal strength/RSSI in dB
    name: "WiFi Signal dB"
    id: wifi_signal_db
    update_interval: 60s
    entity_category: "diagnostic"

  - platform: copy # Reports the WiFi signal strength in %
    source_id: wifi_signal_db
    name: "WiFi Signal Percent"
    filters:
      - lambda: return min(max(2 * (x + 100.0), 0.0), 100.0);
    unit_of_measurement: "Signal %"
    entity_category: "diagnostic"

i2c:
  sda: GPIO33
  scl: GPIO32
  frequency: 400kHz

i2s_isound:
  - i2s_lrclk_pin: GPIO25
    i2s_bclk_pin: GPIO26
    i2s_mclk_pin: GPIO0
    

media_player:
  - platform: i2s_isound
    name: iSound
    id: iSound
    dac_type: external
    i2s_dout_pin: GPIO27
    mute_pin: 
      number: 18
      inverted: true
      mode:
        output: true
        open_drain: true
    mode: mono
