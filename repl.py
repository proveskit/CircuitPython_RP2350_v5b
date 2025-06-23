# This is where the magic happens!
# This file is executed on every boot (including wake-boot from deepsleep)
# Created By: Michael Pham

"""
Built for the PySquared V5a FC Board
Version: X.X.X
Published:
"""

import time

import digitalio
import sdcardio
import storage

try:
    # from board_definitions import proveskit_rp2040_v4 as board
    raise ImportError
except ImportError:
    import board

try:
    from typing import Union
except Exception:
    pass

import os

from pysquared.protos.power_monitor import PowerMonitorProto

import lib.pysquared.functions as functions
import lib.pysquared.nvm.register as register
from lib.adafruit_drv2605 import DRV2605  ### This is Hacky V5a Devel Stuff###
from lib.adafruit_mcp230xx.mcp23017 import (
    MCP23017,  ### This is Hacky V5a Devel Stuff###
)
from lib.adafruit_mcp9808 import MCP9808  ### This is Hacky V5a Devel Stuff###
from lib.adafruit_tca9548a import TCA9548A  ### This is Hacky V5a Devel Stuff###
from lib.adafruit_veml7700 import VEML7700  ### This is Hacky V5a Devel Stuff###

# from lib.pysquared.Big_Data import AllFaces  ### This is Hacky V5a Devel Stuff###
from lib.pysquared.cdh import CommandDataHandler
from lib.pysquared.config.config import Config
from lib.pysquared.hardware.burnwire.manager.burnwire import BurnwireManager
from lib.pysquared.hardware.busio import _spi_init, initialize_i2c_bus
from lib.pysquared.hardware.digitalio import initialize_pin
from lib.pysquared.hardware.imu.manager.lsm6dsox import LSM6DSOXManager
from lib.pysquared.hardware.magnetometer.manager.lis2mdl import LIS2MDLManager
from lib.pysquared.hardware.power_monitor.manager.ina219 import INA219Manager
from lib.pysquared.hardware.radio.manager.rfm9x import RFM9xManager
from lib.pysquared.hardware.radio.manager.sx1280 import SX1280Manager
from lib.pysquared.logger import Logger
from lib.pysquared.nvm.counter import Counter
from lib.pysquared.nvm.flag import Flag
from lib.pysquared.rtc.manager.microcontroller import MicrocontrollerManager
from lib.pysquared.satellite import Satellite
from lib.pysquared.sleep_helper import SleepHelper
from lib.pysquared.watchdog import Watchdog
from version import __version__

rtc = MicrocontrollerManager()

logger: Logger = Logger(
    error_counter=Counter(index=register.ERRORCNT),
    colorized=True,
)

logger.info(
    "Booting",
    hardware_version=os.uname().version,
    software_version=__version__,
)

watchdog = Watchdog(logger, board.WDT_WDI)
watchdog.pet()

logger.debug("Initializing Config")
config: Config = Config("config.json")

mux_reset = initialize_pin(logger, board.MUX_RESET, digitalio.Direction.OUTPUT, False)

## Init Buses ##
# TODO(nateinaction): fix spi init
spi0 = _spi_init(
    logger,
    board.SPI0_SCK,
    board.SPI0_MOSI,
    board.SPI0_MISO,
)

spi1 = _spi_init(
    logger,
    board.SPI1_SCK,
    board.SPI1_MOSI,
    board.SPI1_MISO,
)

i2c0 = initialize_i2c_bus(
    logger,
    board.SCL0,
    board.SDA0,
    100000,
)

i2c1 = initialize_i2c_bus(
    logger,
    board.SCL1,
    board.SDA1,
    100000,
)

## Init Helper Classes ##

c = Satellite(logger, config)

sleep_helper = SleepHelper(c, logger, watchdog, config)

## Init Misc Pins ##
burnwire_heater_enable = initialize_pin(
    logger, board.FIRE_DEPLOY1_A, digitalio.Direction.OUTPUT, False
)
burnwire1_fire = initialize_pin(
    logger, board.FIRE_DEPLOY1_B, digitalio.Direction.OUTPUT, False
)

## Init Hardware ##
# TODO: Replace this with new radio_config rather than the flag
use_fsk_flag = Flag(index=register.FLAG, bit_index=7)
use_fsk_flag.toggle(False)

try:
    radio = RFM9xManager(
        logger,
        config.radio,
        spi0,
        initialize_pin(logger, board.SPI0_CS0, digitalio.Direction.OUTPUT, True),
        initialize_pin(logger, board.RF1_RST, digitalio.Direction.OUTPUT, True),
    )
except Exception as e:
    logger.error("Error Initializing Radio", e)
    radio = None

magnetometer = LIS2MDLManager(logger, i2c1)

imu = LSM6DSOXManager(logger, i2c1, 0x6B)

cdh = CommandDataHandler(config, logger, radio)

f = functions.functions(
    c,
    logger,
    config,
    sleep_helper,
    radio,
    magnetometer,
    imu,
    watchdog,
    cdh,
)
try:
    sd = sdcardio.SDCard(spi1, board.SPI1_CS1)
    vfs = storage.VfsFat(sd)
    storage.mount(vfs, "/sd")
    logger.info("SD Card Mounted Successfully")
except Exception as e:
    logger.error("Error Initializing SD Card", e)
    sd = None

### This is Hacky V5a Devel Stuff###
## Initialize the Second Radio ##

radio2 = SX1280Manager(
    logger,
    config.radio,
    spi1,
    initialize_pin(logger, board.SPI1_CS0, digitalio.Direction.OUTPUT, True),
    initialize_pin(logger, board.RF2_RST, digitalio.Direction.OUTPUT, True),
    initialize_pin(logger, board.RF2_IO0, digitalio.Direction.OUTPUT, True),
    2.4,
    initialize_pin(logger, board.RF2_TX_EN, digitalio.Direction.OUTPUT, True),
    initialize_pin(logger, board.RF2_RX_EN, digitalio.Direction.OUTPUT, True),
)

radio2.send("Hello World")
print("Radio2 sent Hello World")

try:
    battery_power_monitor: PowerMonitorProto = INA219Manager(logger, i2c1, 0x40)
    solar_power_monitor: PowerMonitorProto = INA219Manager(logger, i2c1, 0x44)
except Exception as e:
    logger.error("Error Initializing Power Monitors", e)
    battery_power_monitor = None
    solar_power_monitor = None

## Initializing the Burn Wire ##
antenna_deployment = BurnwireManager(
    logger, burnwire_heater_enable, burnwire1_fire, enable_logic=True
)


## Initializing the Heater ##
def heater_pulse() -> None:
    """
    This function is used to turn on the heater.
    It will turn on the heater for 5 seconds and then turn it off.

    Args:
        None
    Returns:
        None
    """
    ENABLE_HEATER.value = False
    logger.info("Heater Enabled")
    time.sleep(5)
    logger.info("Heater Disabled")
    ENABLE_HEATER.value = True


## Initialize the MCP23017 GPIO Expander and its pins ##
GPIO_RESET = initialize_pin(
    logger, board.GPIO_EXPANDER_RESET, digitalio.Direction.OUTPUT, True
)
print("GPIO Reset Pin Initialized")
mcp = MCP23017(i2c1)
print("MCP23017 Initialized")

# This sets up all of the GPIO pins on the MCP23017
FACE4_ENABLE = mcp.get_pin(8)
FACE0_ENABLE = mcp.get_pin(9)
FACE1_ENABLE = mcp.get_pin(10)
FACE2_ENABLE = mcp.get_pin(11)
FACE3_ENABLE = mcp.get_pin(12)
ENAB_RF = mcp.get_pin(13)
VBUS_RESET = mcp.get_pin(14)
SPI0_CS1 = mcp.get_pin(15)
ENABLE_HEATER = mcp.get_pin(0)
PAYLOAD_PWR_ENABLE = mcp.get_pin(1)
Z_GPIO0 = mcp.get_pin(2)
Z_GPIO1 = mcp.get_pin(3)
RF2_IO2 = mcp.get_pin(4)
RF2_IO1 = mcp.get_pin(5)

print("GPIO Pins Initialized")

# This defines the direction of the GPIO pins
FACE4_ENABLE.direction = digitalio.Direction.OUTPUT
FACE0_ENABLE.direction = digitalio.Direction.OUTPUT
FACE1_ENABLE.direction = digitalio.Direction.OUTPUT
FACE2_ENABLE.direction = digitalio.Direction.OUTPUT
FACE3_ENABLE.direction = digitalio.Direction.OUTPUT
ENAB_RF.direction = digitalio.Direction.OUTPUT
ENAB_RF.value = True  # Enable the RF
VBUS_RESET.direction = digitalio.Direction.OUTPUT
VBUS_RESET.value = True  # Enable the VBUS Reset
ENABLE_HEATER.direction = digitalio.Direction.OUTPUT
PAYLOAD_PWR_ENABLE.direction = digitalio.Direction.OUTPUT

print("GPIO Pins Direction Set")


# Face Control Helper Functions
def all_faces_off():
    """
    This function turns off all of the faces. Note the load switches are disabled high.
    """
    FACE0_ENABLE.value = False
    FACE1_ENABLE.value = False
    FACE2_ENABLE.value = False
    FACE3_ENABLE.value = False
    FACE4_ENABLE.value = False


def all_faces_on():
    """
    This function turns on all of the faces. Note the load switches are enabled low.
    """
    FACE0_ENABLE.value = True
    FACE1_ENABLE.value = True
    FACE2_ENABLE.value = True
    FACE3_ENABLE.value = True
    FACE4_ENABLE.value = True


## Face Sensor Stuff ##

# This is the TCA9548A I2C Multiplexer
all_faces_on()

time.sleep(0.1)  # Wait for the faces to power on
mux_reset.value = True  # Reset the multiplexer

tca = TCA9548A(i2c1, address=int(0x77))


### This is Hacky V5a Devel Stuff###
class Face:
    def __init__(self, add: int, pos: str, tca: TCA9548A, logger: Logger) -> None:
        self.tca: TCA9548A = tca
        self.address: int = add
        self.position: str = pos
        self.logger: Logger = logger

        # Use tuple instead of list for immutable data
        self.senlist: tuple = ()
        # Define sensors based on position using a dictionary lookup instead of if-elif chain
        sensor_map: dict[str, tuple[str, ...]] = {
            "x+": ("MCP", "VEML", "DRV"),
            "x-": ("MCP", "VEML"),
            "y+": ("MCP", "VEML", "DRV"),
            "y-": ("MCP", "VEML"),
            "z-": ("MCP", "VEML", "DRV"),
        }
        self.senlist: tuple[str, ...] = sensor_map.get(pos, ())

        # Initialize sensor states dict only with needed sensors
        self.sensors: dict[str, bool] = {sensor: False for sensor in self.senlist}

        # Initialize sensor objects as None
        self.mcp = None
        self.veml = None
        self.drv = None

    def sensor_init(self, senlist, address) -> None:
        if "MCP" in senlist:
            try:
                self.mcp: MCP9808 = MCP9808(self.tca[address], address=27)
                self.sensors["MCP"] = True
            except Exception as e:
                self.logger.error("Error Initializing Temperature Sensor", e)

        if "VEML" in senlist:
            try:
                self.veml: VEML7700 = VEML7700(self.tca[address])
                self.sensors["VEML"] = True
            except Exception as e:
                self.logger.error("Error Initializing Light Sensor", e)

        if "DRV" in senlist:
            try:
                self.drv: DRV2605 = DRV2605(self.tca[address])
                self.sensors["DRV"] = True
            except Exception as e:
                self.logger.error("Error Initializing Motor Driver", e)


class AllFaces:
    def __init__(self, tca: TCA9548A, logger: Logger) -> None:
        self.tca: TCA9548A = tca
        self.faces: list[Face] = []
        self.logger: Logger = logger

        # Create faces using a loop instead of individual variables
        positions: list[tuple[str, int]] = [
            ("y+", 0),
            ("y-", 1),
            ("x+", 2),
            ("x-", 3),
            ("z-", 4),
        ]
        for pos, addr in positions:
            face: Face = Face(addr, pos, tca, self.logger)
            face.sensor_init(face.senlist, face.address)
            self.faces.append(face)

    def face_test_all(self) -> list[list[float]]:
        results: list[list[float]] = []
        for face in self.faces:
            if face:
                try:
                    temp: Union[float, None] = (
                        face.mcp.temperature if face.sensors.get("MCP") else None
                    )
                    light: Union[float, None] = (
                        face.veml.lux if face.sensors.get("VEML") else None
                    )
                    results.append([temp, light])
                except Exception:
                    results.append([None, None])
        return results


all_faces = AllFaces(tca, logger)

## Onboard Temp Sensor ##
mcp = MCP9808(i2c1, address=30)  # Not working for some reason
