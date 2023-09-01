# Manual Control Using a Joystick

The `blue_joy` package has been configured to enable manual control of a
robot using a joystick. Manual control can be accomplished regardless of
whether Blue is running on a topside computer or on the vehicle itself (as
long as a network connection exists).

## Configuring a joystick with Blue running on the topside

The easiest way to accomplish manual control with Blue is when Blue is running
on a topside computer. This configuration has the least restrictions on the
type of controller used as well. To enable joystick control from a topside
computer, simply install the `joystick` apt package:

```bash
sudo apt install joystick
```

Once this package has been installed, confirm that you are able to observe
inputs from an attached controller by running

```bash
jstest /dev/input/<your-joystick-device>
```

## Configuring a joystick with Blue running on the robot

Joystick control can be accomplished with Blue running on the ROV. While
enabling manual control in this configuration requires a more involved setup
process, it significantly reduces the amount of data being sent over the
tether. However, there are some restrictions regarding the joysticks
supported. The controller that we use for testing is the [Logitech F310](https://www.logitechg.com/en-us/products/gamepads/f310-gamepad.940-000110.html).
There are known issues with USB/IP and XBox controllers that make configuring
XBox controllers quite a bit more challenging to setup.

### Dependencies

To accomplish manual control with Blue running on the ROV, start by
installing the following dependencies onto both the topside system that the
joystick is connected to and onto the ROV itself where Blue is running.

```bash
sudo apt install linux-tools-generic hwdata joystick
```

### Topside setup

We will now configure the topside system for manual control. First, load the
USB/IP kernel driver

```bash
sudo modprobe usbip-host
```

Next, start the USB/IP daemon

```bash
sudo usbipd &
```

> NOTE: USB/IP gets installed into directories that are not in your `$PATH`.
Because of this, it creates symlinks in the `/usr/bin` directory. Sometimes you will get an error when using the default symlinks indicating
that the installed `linux-tools-generic` version does not match the kernel. If this happens,
simply delete the existing symlinks and replace them with new ones.

With your joystick connected to the topside computer, check that USB/IP is
able to recognize the joystick

```bash
usbip list -l
```

You should observe your joystick in the devices listed along with its bus ID.
As an example, the Logitech F310 may be listed as

```bash
 - busid 3-4 (046d:c216)
   Logitech, Inc. : F310 Gamepad [DirectInput Mode] (046d:c216)
```

where it can be seen with a bus ID of `3-4`.

With the joystick visible to USB/IP, we now bind the joystick

```bash
sudo usbip bind -b <bus-id>
```

You can confirm that the device has been bound by verifying that it
is listed as an exportable device

```bash
usbip list -r 127.0.0.1
```

which should yield an output similar to

```bash
Exportable USB devices
======================
 - 127.0.0.1
        3-4: Logitech, Inc. : F310 Gamepad [DirectInput Mode] (046d:c216)
           : /sys/devices/pci0000:00/0000:00:01.2/0000:20:00.0/0000:21:08.0/0000:2a:00.3/usb3/3-4
           : (Defined at Interface level) (00/00/00)

```

### Robot setup

To configure manual control on the robot, start by loading the `vhci-hcd` module.

```bash
sudo modprobe vhci-hcd
```

Next, check that the joystick is observable by the robot

```bash
sudo usbip list -r <topside-ip-address>
```

After confirming that the joystick device is exportable, attach to it from the robot

```bash
sudo usbip attach -r <topside-ip-address> -b <bus-id>
```

The joystick should now be exposed on the robot as if it were plugged directly into the robot. You can confirm that the joystick works by testing it with `jstest`

```bash
jstest /dev/input/<your-joystick-device>
```

### Shutdown

To shutdown the system, first identify the port at which the robot has established a connection with the topside computer at

```bash
sudo usbip port
```

Now detatch the port corresponding to the joystick device

```bash
sudo usbip detach -p <port>
```

Finally, unbind the device from the topside computer

```bash
sudo usbip unbind -b <bus-id>
```
