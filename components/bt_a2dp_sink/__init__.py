"""
ESPHome External Component — bt_a2dp_sink
=========================================
Exposes a native ESP-IDF Bluetooth Classic A2DP sink that pipes
decoded PCM audio directly into an ESPHome speaker component.

Config keys:
  speaker        (required) — ESPHome speaker that receives PCM data.
                              Route to media_resampling_speaker so BT
                              audio goes through the resampler → mixer
                              → I2S chain and the TAS58xx DSP/EQ.
  media_player   (required) — Media player to stop when BT connects
                              (prevents I2S contention with Sendspin).
  device_name    (optional) — Bluetooth device name shown on phone.
                              Default: "Audio Brick"
  connected_sensor (optional) — binary_sensor to publish BT state to.
  dac_switch     (optional) — switch to turn on when BT connects so
                              the DAC is always active during playback.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import speaker, media_player, binary_sensor, switch
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32", "speaker"]
AUTO_LOAD = []

bt_a2dp_ns = cg.esphome_ns.namespace("bt_a2dp_sink")
BTA2DPSinkComponent = bt_a2dp_ns.class_("BTA2DPSinkComponent", cg.Component)

CONF_SPEAKER = "speaker"
CONF_MEDIA_PLAYER = "media_player"
CONF_DEVICE_NAME = "device_name"
CONF_CONNECTED_SENSOR = "connected_sensor"
CONF_DAC_SWITCH = "dac_switch"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BTA2DPSinkComponent),
        cv.Required(CONF_SPEAKER): cv.use_id(speaker.Speaker),
        cv.Required(CONF_MEDIA_PLAYER): cv.use_id(media_player.MediaPlayer),
        cv.Optional(CONF_DEVICE_NAME, default="Audio Brick"): cv.string,
        cv.Optional(CONF_CONNECTED_SENSOR): cv.use_id(
            binary_sensor.BinarySensor
        ),
        cv.Optional(CONF_DAC_SWITCH): cv.use_id(switch.Switch),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    spk = await cg.get_variable(config[CONF_SPEAKER])
    cg.add(var.set_speaker(spk))

    mp = await cg.get_variable(config[CONF_MEDIA_PLAYER])
    cg.add(var.set_media_player(mp))

    cg.add(var.set_device_name(config[CONF_DEVICE_NAME]))

    if CONF_CONNECTED_SENSOR in config:
        sens = await cg.get_variable(config[CONF_CONNECTED_SENSOR])
        cg.add(var.set_connected_sensor(sens))

    if CONF_DAC_SWITCH in config:
        sw = await cg.get_variable(config[CONF_DAC_SWITCH])
        cg.add(var.set_dac_switch(sw))
