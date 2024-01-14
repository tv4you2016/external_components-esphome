import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import BME68xBSECComponent, CONF_BME68X_BSEC_ID

DEPENDENCIES = ["bme68x_bsec"]

ICON_ACCURACY = "mdi:checkbox-marked-circle-outline"

TYPES = []

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BME68X_BSEC_ID): cv.use_id(BME68xBSECComponent),
    }
)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BME68X_BSEC_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
