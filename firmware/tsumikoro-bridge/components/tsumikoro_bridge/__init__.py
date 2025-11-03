"""
Tsumikoro Bridge Custom Component for ESPHome

This component provides a bridge between ESPHome (ESP32) and Tsumikoro
motor controllers (STM32), enabling network control and monitoring.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

tsumikoro_bridge_ns = cg.esphome_ns.namespace("tsumikoro_bridge")
TsumikoroBridge = tsumikoro_bridge_ns.class_(
    "TsumikoroBridge", cg.Component, uart.UARTDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TsumikoroBridge),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    """Generate C++ code for the component."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
