# J773g DockChannel console: source review and test plan

**Status, 2026-11-02:** a Gravity-based m1n1 stage 2 on an M4 Mac mini (J773g, T8132) initialized its DockChannel UART at `0x388128000`. That is a firmware observation; external serial output, Linux console output, and an m1n1 proxy connection have **not** been demonstrated on this machine. Avoid changing the installed boot chain merely to try this plan.

## Port and cable

For the 2024 Apple-silicon Mac mini, [Apple's current DFU-port table](https://support.apple.com/en-us/120694) identifies the **leftmost rear USB-C port when facing the back**. Some older J773g notes describe a different port; verify the target model before connecting. The table identifies a DFU port; it does not establish serial routing, which has not been physically tested here.

On an Apple-silicon Mac host, connect its model's DFU port to the Mini's DFU port. For `macvdmtool serial`, the [upstream tool documentation](https://github.com/AsahiLinux/macvdmtool/blob/b22ae51eb43a0e1daa21d41616ac899f28e7bf8a/README.md) specifies a SuperSpeed-capable USB-C data cable; USB-2-only charge cables lack the required pins. The tool documents different cable requirements for DebugUSB. Apple's separate firmware-revive procedure has its own cable rules; do not assume those instructions are interchangeable with serial debugging.

## Candidate connection

1. Build [AsahiLinux/macvdmtool](https://github.com/AsahiLinux/macvdmtool) on the Apple-silicon macOS host with Xcode command-line tools and `make`.
2. With both machines idle and the test coordinated, connect the cable and run `sudo ./macvdmtool serial` on the host. This changes port modes; it does not itself invoke the tool's reboot command. Its common setup can reset a host port controller if unlocking fails, so a connected device may be disturbed. Do not substitute `reboot serial` without deliberately planning a reboot.
3. Inspect `/dev/cu.*` for the host's debug console. The upstream README uses both `cu.debug_console` and `cu.debug-console` spellings, so select the actual device; its existence alone does not prove that the Mini is connected or producing data. An initial `screen <device> 115200` capture is a candidate, not a validated J773g setup.
4. A m1n1 proxy needs a target actually waiting in proxy mode and a matching `M1N1DEVICE`. A normal boot that has already passed m1n1 stage 2 does not automatically provide one. DebugUSB, m1n1 USB CDC, and the hypervisor are distinct paths; their availability on this target has not been established by this test.

[Asahi kisd](https://github.com/AsahiLinux/kisd#base-addresses) lists T8132 DebugUSB/DockChannel support as verified for its Linux-host tooling. That does not verify macOS-to-J773g serial or proxy operation. The inspected [Aurora](https://github.com/aurora-silicon/m1n1/blob/cf57aebc36b2/src/usb.c) and [omacom](https://github.com/omacom/m1n1/blob/bc79acc434e6/src/usb.c) m1n1 trees contain USB CDC support with specialized T8140 paths; their presence alone does not establish J773g support.

## Candidate Linux early console

[Aurora 7.3's T8132 DT](https://github.com/aurora-silicon/linux/blob/19f71a142419e0393efdd4d7845c29a560eb1b19/arch/arm64/boot/dts/apple/t8132.dtsi) declares a DockChannel UART data window at `0x38812c000`, distinct from the `0x388128000` configuration window. Its [serial driver](https://github.com/aurora-silicon/linux/blob/19f71a142419e0393efdd4d7845c29a560eb1b19/drivers/tty/serial/apple_dockchannel_uart.c) supports earlycon and `ttyDC`. A **source-supported, untested** test-kernel command line is:

```text
earlycon=dockchannel,0x38812c000 console=ttyDC0 keep_bootcon
```

Before testing, verify the final kernel has `CONFIG_SERIAL_EARLYCON=y`, `CONFIG_SERIAL_APPLE_DOCKCHANNEL_EARLYCON=y`, and `CONFIG_SERIAL_APPLE_DOCKCHANNEL=y`, and that the final DTB enables the expected UART node and interrupt. Verify the external connection and coordinate the boot first. Source support and an inherited DTS node are not a successful console test.

The inspected [Gravity early-UART implementation](https://github.com/GravityLinux/linux/blob/4fc326f6c25ccceb7515e2635c22e01fde5d3bac/drivers/tty/serial/apple_dockchannel_early_uart.c) waits for transmit space without a timeout; enabling it without a draining receiver could hang a boot. Do not add this option to an existing default boot without validating the receiver and recovery path.
