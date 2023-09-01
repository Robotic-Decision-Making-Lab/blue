# USB/IP Setup

This document describes my testing with `usbip` for joystick forwarding

## Installation

Install dependencies prior to getting started:

```bash
sudo apt install linux-tools-generic hwdata
```

## Server-Side Setup

1. Load the USB/IP kernel driver

```bash
sudo modprobe usbip_host
```

2. Start the USB/IP daemon

```bash
sudo usbipd &
```

> NOTE: If you get an error when running the above command mentioning that
the version of the installed tool is incompatible with the kernel, you can
delete the existing symlinks, and create new ones and the error should go
away:

```bash
sudo rm -rf /usr/bin/usbip && rm -rf /usr/bin/usbipd
sudo ln -s /usr/lib/linux-tools/<version-number>/usbip /usr/bin/usbip
sudo ln -s /usr/lib/linux-tools/<version-number>/usbipd /usr/bin/usbipd
```

3. List all devices connected:

```bash
usbip list -l
```

which should return something like

```bash
<insert example>
```

4. Select and share the device that you want to bind:

```bash
sudo usbip bind -b <device-id>
```

5. You can confirm that the device has been bound by checking the exportable
devices

```bash
usbip list -r 127.0.0.1
```
