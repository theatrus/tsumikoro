"""
Tsumikoro Bridge Custom Component for ESPHome

This component provides a bus controller for the Tsumikoro motor control bus,
using the full bus protocol stack with FreeRTOS threading.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_BAUD_RATE
from esphome import pins

# No longer depends on uart component - we use ESP-IDF UART directly
DEPENDENCIES = []

# Configuration keys
CONF_UART_PORT = "uart_port"
CONF_TX_PIN = "tx_pin"
CONF_RX_PIN = "rx_pin"
CONF_DE_PIN = "de_pin"
CONF_DEVICE_ID = "device_id"
CONF_TURNAROUND_DELAY = "turnaround_delay"

tsumikoro_bridge_ns = cg.esphome_ns.namespace("tsumikoro_bridge")
TsumikoroBridge = tsumikoro_bridge_ns.class_(
    "TsumikoroBridge", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TsumikoroBridge),
        cv.Required(CONF_UART_PORT): cv.int_range(min=0, max=2),
        cv.Required(CONF_TX_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_RX_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_DE_PIN, default=-1): pins.gpio_output_pin_schema,
        cv.Optional(CONF_BAUD_RATE, default=1000000): cv.int_range(min=9600, max=2000000),
        cv.Optional(CONF_DEVICE_ID, default=0x00): cv.hex_uint8_t,
        cv.Optional(CONF_TURNAROUND_DELAY, default=3): cv.int_range(min=0, max=10),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    """Generate C++ code for the component."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Set UART configuration
    cg.add(var.set_uart_port(config[CONF_UART_PORT]))
    # Extract pin numbers from pin config objects
    tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_tx_pin(tx_pin.get_pin()))
    rx_pin = await cg.gpio_pin_expression(config[CONF_RX_PIN])
    cg.add(var.set_rx_pin(rx_pin.get_pin()))
    de_pin = await cg.gpio_pin_expression(config[CONF_DE_PIN])
    cg.add(var.set_de_pin(de_pin.get_pin()))
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))

    # Set bus configuration
    cg.add(var.set_device_id(config[CONF_DEVICE_ID]))
    cg.add(var.set_turnaround_delay(config[CONF_TURNAROUND_DELAY]))
