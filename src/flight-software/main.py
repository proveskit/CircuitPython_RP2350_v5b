import gc
import os
import time

import board
import digitalio
import microcontroller
from lib.adafruit_mcp230xx.mcp23017 import MCP23017
from lib.adafruit_tca9548a import TCA9548A
from lib.proveskit_rp2350_v5b.register import Register
from lib.pysquared.beacon import Beacon
from lib.pysquared.cdh import CommandDataHandler
from lib.pysquared.config.config import Config
from lib.pysquared.hardware.busio import _spi_init, initialize_i2c_bus
from lib.pysquared.hardware.digitalio import initialize_pin
from lib.pysquared.hardware.imu.manager.lsm6dsox import LSM6DSOXManager
from lib.pysquared.hardware.magnetometer.manager.lis2mdl import LIS2MDLManager
from lib.pysquared.hardware.power_monitor.manager.ina219 import INA219Manager
from lib.pysquared.hardware.radio.manager.rfm9x import RFM9xManager
from lib.pysquared.hardware.radio.manager.sx1280 import SX1280Manager
from lib.pysquared.hardware.radio.packetizer.packet_manager import PacketManager
from lib.pysquared.logger import Logger
from lib.pysquared.nvm.counter import Counter
from lib.pysquared.protos.power_monitor import PowerMonitorProto
from lib.pysquared.rtc.manager.microcontroller import MicrocontrollerManager
from version import __version__

boot_time: float = time.time()

rtc = MicrocontrollerManager()

(boot_count := Counter(index=Register.boot_count)).increment()
error_count: Counter = Counter(index=Register.error_count)

logger: Logger = Logger(
    error_counter=error_count,
    colorized=False,
)

logger.info(
    "Booting",
    hardware_version=os.uname().version,  # type: ignore[attr-defined]
    software_version=__version__,
)

logger.debug("Initializing Config")
config: Config = Config("config.json")

loiter_time: int = 5
for i in range(loiter_time):
    logger.info(f"Code Starting in {loiter_time-i} seconds")
    time.sleep(1)

try:
    SPI0_CS0 = initialize_pin(logger, board.SPI0_CS0, digitalio.Direction.OUTPUT, True)
    SPI1_CS0 = initialize_pin(logger, board.SPI1_CS0, digitalio.Direction.OUTPUT, True)
    GPIO_RESET = initialize_pin(
        logger, board.GPIO_EXPANDER_RESET, digitalio.Direction.OUTPUT, True
    )

    i2c1 = initialize_i2c_bus(
        logger,
        board.SCL1,
        board.SDA1,
        100000,
    )

    i2c0 = initialize_i2c_bus(
        logger,
        board.SCL0,
        board.SDA0,
        100000,
    )

    mcp = MCP23017(i2c1)

    # GPB
    FACE4_ENABLE = mcp.get_pin(8)
    FACE0_ENABLE = mcp.get_pin(9)
    FACE1_ENABLE = mcp.get_pin(10)
    FACE2_ENABLE = mcp.get_pin(11)
    FACE3_ENABLE = mcp.get_pin(12)
    FACE5_ENABLE = mcp.get_pin(13)

    # GPA
    ENABLE_HEATER = mcp.get_pin(0)
    PAYLOAD_PWR_ENABLE = mcp.get_pin(1)
    FIRE_DEPLOY2_B = mcp.get_pin(2)
    PAYLOAD_BATT_ENABLE = mcp.get_pin(3)
    RF2_IO2 = mcp.get_pin(4)
    RF2_IO1 = mcp.get_pin(5)
    RF2_IO0 = mcp.get_pin(6)
    RF2_IO3 = mcp.get_pin(7)

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

    sband_radio = SX1280Manager(
        logger,
        config.radio,
        spi1,
        SPI1_CS0,
        initialize_pin(logger, board.RF2_RST, digitalio.Direction.OUTPUT, True),
        RF2_IO0,
        2.4,
        initialize_pin(logger, board.RF2_TX_EN, digitalio.Direction.OUTPUT, False),
        initialize_pin(logger, board.RF2_RX_EN, digitalio.Direction.OUTPUT, False),
    )

    uhf_radio = RFM9xManager(
        logger,
        config.radio,
        spi0,
        SPI0_CS0,
        initialize_pin(logger, board.RF1_RST, digitalio.Direction.OUTPUT, True),
    )

    magnetometer = LIS2MDLManager(logger, i2c1)

    imu = LSM6DSOXManager(logger, i2c1, 0x6B)

    uhf_packet_manager = PacketManager(
        logger,
        uhf_radio,
        config.radio.license,
        Counter(2),
        0.2,
    )

    cdh = CommandDataHandler(logger, config, uhf_packet_manager)

    beacon = Beacon(
        logger,
        config.cubesat_name,
        uhf_packet_manager,
        time.monotonic(),
        imu,
        magnetometer,
        uhf_radio,
        sband_radio,
    )

    # Face Control Helper Functions
    def all_faces_off():
        """
        This function turns off all of the faces. Note the load switches are disabled low.
        """
        FACE0_ENABLE.value = False
        FACE1_ENABLE.value = False
        FACE2_ENABLE.value = False
        FACE3_ENABLE.value = False
        FACE4_ENABLE.value = False

    def all_faces_on():
        """
        This function turns on all of the faces. Note the load switches are enabled high.
        """
        FACE0_ENABLE.value = True
        FACE1_ENABLE.value = True
        FACE2_ENABLE.value = True
        FACE3_ENABLE.value = True
        FACE4_ENABLE.value = True

    mux_reset = initialize_pin(
        logger, board.MUX_RESET, digitalio.Direction.OUTPUT, False
    )
    all_faces_on()
    time.sleep(0.1)
    mux_reset.value = True
    tca = TCA9548A(i2c0, address=int(0x77))  # all 3 connected to high

    battery_power_monitor: PowerMonitorProto = INA219Manager(logger, i2c0, 0x40)
    solar_power_monitor: PowerMonitorProto = INA219Manager(logger, i2c0, 0x41)

    def nominal_power_loop():
        logger.debug(
            "FC Board Stats",
            bytes_remaining=gc.mem_free(),
        )

        all_faces_on()

        uhf_packet_manager.send(config.radio.license.encode("utf-8"))

        beacon.send()

        cdh.listen_for_commands(10)

        beacon.send()

        cdh.listen_for_commands(config.sleep_duration)

    try:
        logger.info("Entering main loop")
        while True:
            # TODO(nateinaction): Modify behavior based on power state
            nominal_power_loop()

    except Exception as e:
        logger.critical("Critical in Main Loop", e)
        time.sleep(10)
        microcontroller.on_next_reset(microcontroller.RunMode.NORMAL)
        microcontroller.reset()
    finally:
        logger.info("Going Neutral!")

except Exception as e:
    logger.critical("An exception occured within main.py", e)
