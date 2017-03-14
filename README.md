# Trashbot

This project is split into separate modules:
* Kinect sensor model
* Rexarm controller
* MagicBot motion controller
* Joystick
each of which runs on its own process. To build any of these executables, perform
the following steps.

## Steps to build:

1. `cd <module>`
2. `install_deps.sh`
3. `source set_ld_path.sh`
4. `make`
