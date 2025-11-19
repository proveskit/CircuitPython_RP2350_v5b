# FC V5b Quickstart Guide
This is a quick guide that can be used to quickstart and test the core functions of the V5b Flight Controller Board!

## Firmware and Software Compatibilities
The V5b FC board uses the standard PROVES Kit software development toolchain. At this time it is cross compatible with V5a firmware but has some key incompatibilities with V5a software.

You can install firmware using the manual process (drag and drop onto the RP2350 boot drive) or you can try using the following terminal command (note this command currently installs V5a firmware):
```sh
make install-firmware
```

You can install software by running the following command in the root directory of this repo:
```sh
make install BOARD_MOUNT_POINT="Enter the Board Mount Point Here"
```

If you want to use V5a software on a V5b board note that the enable levels of *all* the load switches are inverted. This means that although V5b boards enable their load switches by setting their `value` to `True` V5a boards enable their load switches by setting their `value` to `False`. This can cause some surprises if not carefully checked.

> NOTE: V5b Flight Controller Boards are still affected by the bug with the deep sleep implementation for RP2350's in CircuitPython. If your board appears to "disappear" soon after booting simply power cycle and attempt to establish a serial connection ASAP to get around this issue.

## Individual Functional Checkouts
At the moment the best way to verify individual hardware functionalities is to enter the `REPL` by interrupting any running code and allowing `reply.py` to setup manual test functions. You can interrupt the `main` loop with a `ctrl+c` terminal command. Note this appears to work significantly better with a dedicated serial terminal software like `Tabby` or `Warp` rather than the built in serial terminals in `VS Code` or `Cursor`.

Once you're in the `REPL` you'll have access to the following objects:
- `watchdog` | This component handles interaction with the external rad tolerant watchdog.
- `logger` | This component takes function outputs and formats them in `json`. Currently just prints to terminal but will soon support saving logs to the Flash Memory.
- `config` | This component interacts with `config.json` where satellite parameters are set
- `c` | The satellite helper class that handles resets, uptime, and mode switching.
- `sleep_helper` | This helper class provides access to `deep_sleep` functions. It doesn't really do anything in the REPL because the USB connection is maintained blocking deep sleep.
- `radio` | This radio is the default 437.4 MHz RFM98PW amateur band UHF radio.
- `magnetometer` | This component encapsulates interacting with the onboard magnetometer.
- `imu` | This component interacts with the 6-dof accel and gyro Inertial Measurement Unit.
- `cdh` | This is a helper class for parsing incoming packets and commands over the radio.
- `f` | This is the legacy functions helper class.
- `radio2` | This radio is the 2.4Ghz S-Band SX1281 Radio.
- `mcp` | The MCP GPIO expander, not yet added to a dedicated manager class
- `tca`| The TCA I2C Multiplexer, not yet added to a dedicated manager class
- `all_faces` | A scaffold implementation of interacting with data from all of the satellite solar panel faces

Ideally all of the functions in all of these objects should execute without errors when the FC board is fully connected with all other boards in the PROVES Kit. You can learn more about what functions are available in each object by either inspecting them directly or using the following pattern in the `REPL`:

```py
help(OBJECT_NAME)
```
