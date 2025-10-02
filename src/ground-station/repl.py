import board
import digitalio
from busio import SPI
from lib.ground_station.ground_station import GroundStation
from lib.pysquared.cdh import CommandDataHandler
from lib.pysquared.config.config import Config
from lib.pysquared.hardware.busio import _spi_init
from lib.pysquared.hardware.digitalio import initialize_pin
from lib.pysquared.hardware.radio.manager.rfm9x import RFM9xManager
from lib.pysquared.hardware.radio.packetizer.packet_manager import PacketManager
from lib.pysquared.logger import Logger
from lib.pysquared.nvm.counter import Counter

logger: Logger = Logger(
    error_counter=Counter(1),
    colorized=False,
)
config: Config = Config("config.json")

spi0: SPI = _spi_init(
    logger,
    board.SPI0_SCK,
    board.SPI0_MOSI,
    board.SPI0_MISO,
)

radio = RFM9xManager(
    logger,
    config.radio,
    spi0,
    initialize_pin(logger, board.SPI0_CS0, digitalio.Direction.OUTPUT, True),
    initialize_pin(logger, board.RF1_RST, digitalio.Direction.OUTPUT, True),
)

packet_manager = PacketManager(
    logger,
    radio,
    config.radio.license,
    Counter(2),
    0.2,
)

cdh = CommandDataHandler(
    logger,
    config,
    packet_manager,
)

ground_station = GroundStation(
    logger,
    config,
    packet_manager,
    cdh,
)

ground_station.run()
