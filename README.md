# fingervu
FingerVU touch and IR/RC/keys linux kernel driver

## Install

To compile and install the module to the active kernel:

    make
    make install

## Usage

### Module parameters

* `debug`: Print debug messages
  * values: 0 (no), 1 (yes)
  * default: 0 (no)
* `mirror_x`: Mirror X asis
  * values: 0 (no), 1 (yes)
  * default: 0 (no)
* `mirror_y`: Mirror Y axis
  * values: 0 (no), 1 (yes)
  * default: 0 (no)
* `disable_touch`: Disable touchscreen
  * values: 0 (no), 1 (yes)
  * default: 0 (no)
* `disable_idev`: Disable input device
  * values: 0 (no), 1 (yes)
  * default: 0 (no)
* `repeat_delay`: Delay after which a repeated key event will be generated
  * values: [0 .. UINT_MAX] (in ms)
  * default: 660 ms
* `repeat_rate`: Max. repeats per second
  * values: [0 .. UINT_MAX] (in repeats/sec)
  * default: 10 repeats/sec


* Note:

  **Volume up/down**: only `repeat_rate` is used, `repeat_delay` is ignored

### udev-rules
To get persistent device names the following two udev-rules should be used:

    SUBSYSTEM=="input", ENV{ID_INPUT_TOUCHSCREEN}=="?*", ATTRS{idVendor}=="15c2", ATTRS{idProduct}=="30c0", GROUP="users",MODE="0660", SYMLINK+="input/touchscreen"
    SUBSYSTEM=="input", ENV{ID_INPUT_KEY}=="?*", ATTRS{idVendor}=="15c2", ATTRS{idProduct}=="30c0", GROUP="users",MODE="0660", SYMLINK+="input/sg_keyevents"

#### Notes
* `GROUP`: replace `users` through a different group-name if needed/desired
* `idProduct`: replace `30c0` through `3480` if using **FingerVu 706**
