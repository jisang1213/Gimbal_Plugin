# Gimbal Plugin for Raisin

A plugin for gimbal communication that integrates seamlessly with Raisin.

advance() should be called at 100Hz (adjustable in params file)

To send rpy commands, publish to 'gimbal_command'

The joint state is published to: 'gimbal_joint_state'
The left camera transformation is published to 'transforms/d435i_front_L'
The right camera transformation is published to 'transforms/d435i_front_R'

The portname should be fixed to '/dev/gimbal'. Edit the udev rules as follows:
```bash
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

Add the following line:
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="367C344F3132", SYMLINK+="gimbal", MODE="0777"

Then reload rules
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
##Logging
Log directory can be specified in params
Four log files are generated:
- angular_vel.csv
- command.csv
- rpy.csv
- joint_state.csv