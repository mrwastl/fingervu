# fingervu
FingerVU touch and IR/RC/keys linux kernel driver

## Install

To compile and install the module to the active kernel:

    make
    make install


## udev-rules
To get persistent device names the following two udev-rules should be used:

    SUBSYSTEM=="input", ENV{ID_INPUT_TOUCHSCREEN}=="?*", ATTRS{idVendor}=="15c2", ATTRS{idProduct}=="30c0", GROUP="users",MODE="0660", SYMLINK+="input/touchscreen"
    SUBSYSTEM=="input", ENV{ID_INPUT_KEY}=="?*", ATTRS{idVendor}=="15c2", ATTRS{idProduct}=="30c0", GROUP="users",MODE="0660", SYMLINK+="input/sg_keyevents"

### Notes
* `GROUP`: replace `users` through a different group-name if needed/desired
* `idProduct`: replace `30c0` through `3480` if using **FingerVu 706**

