# Chromatic Dumper

Originally posted at https://www.reddit.com/r/ModRetroChromatic/comments/1t5r6as/using_a_modretro_chromatic_as_a_dumper/

## Status

Works, but not friendly. Stop reading if you're not happy using source code and a terminal.

This is good enough for me, I'm sharing in case another developer is interested in picking this up and making it user-friendly. I don't plan to spend more time on this.

## What I've done

* Modified the chromatic FPGA firmware to implement the FlashGBX protocol: [https://github.com/fredemmott/chromatic\_dumper](https://github.com/fredemmott/chromatic_dumper) \- this was Claude, followed by a lot of human debugging/cleanup.
* Modified FlashGBX to recognize the Chromatic with this firmware: https://github.com/Lesserkuma/FlashGBX/pull/128 ; no LLM, based on the existing joey jr support 

Works:

* Chromatic v2
* Dumping OG cartridges (ROM and saves)
* Dumping ModRetro cartridges (ROM and saves)
* *NEW*: Restoring ModRetro cartridges (ROM and saves)
* *NEW*: Dumping and restoring MBC3000 v4 flash carts (ROM and saves)

Does not work:

* ~~MBC3000v4~~ Fixed!

Unknown:

- everything else

## Current flow

Once you've built FlashGBX with those changes:

1. Turn off your Chromatic
2. Insert cartridge
3. Plug in to USB
4. Turn on
5. Load firmware into the FPGA SRAM - **NOT** ROM - this way, nothing's permanent. To get back the original firmware, just turn it off and on again
6. Run the modified FlashGBX

To change cartridges, you need to go back to step 1: turn it off, load the firmware, etc. DO NOT CHANGE CARTRIDGES WHILE THE CHROMATIC HAS POWER.

# What needs doing

Extend the FlashGBX code to detect other firmware and automatically load the modified firmware if needed.

Installer/exe for modified flashgbx

## Loading the firmware

# With GoWin Programmer

You need the full GoWin EDA/Programmer; the license is free, but it's a 'fill in a form and wait a while' thing. The educational version does not include support for this board.

Use `programmer_cli.exe --help` first to check that:

* `--cable-index 0` means GWU2X
* `--operation_index 2` means 'SRAM Program' 

`--help`provides other options like cable by name, but they don't appear to actually work in v1.9.12.02.

    programmer_cli.exe --device GW5A-25A --operation_index 2 --fsFile C:\path\to\evt1_x2.fs --cable-index 0

Replace `C:\path\to\evt1\_x2.fs` with the path to the file from [https://github.com/fredemmott/chromatic\_dumper/releases/latest](https://github.com/fredemmott/chromatic_dumper/releases/latest) , or the output of the build if you used GoWin EDA

## With OpenFPGALoader

On Windows at least, use MRUpdater/cart clinic at least once to make sure you've got the right drivers.

If you're on Windows and already use msys2, follow Openfpgaloader's instructions to install. Otherwise, the easiest way to get a usable windows version is to use pyinstextractor-ng to grab openfpgaloader.exe from the MRUpdater.exe .

    openFPGALoader --cable gwu2x --write-sram C:\path\to\evt1_x2.fs

## Advice for devs

if you want to pick this up, it's probably best to bundle openfpgaloader and the firmware in a modified flashgbx. Not sure about the license on modretro's binary, so for windows, this might mean figuring out how to make a visual studio build yourself. If I were doing this, I'd probably make a fork using vcpkg for dependencies, and use a -static triplet, especially as it already uses CMake.

Given the firmware is only going to FPGA SRAM, I don't think flashgbx's firmware update flow makes sense here; I'd put it in the 'connect' functionality and just do it silently.

You *could* streamline the flow by writing to ROM and requiring a flash back, but to me, it's great for safety that you just need to turn the thing off and on again to get back to ModRetro's firmware.

---

# Chromatic FPGA
This repository houses the ModRetro Chromatic's FPGA design files.

For more information about the ModRetro Chromatic, please see visit [ModRetro.com](https://modretro.com/).

## Setup

### Repository

This project builds upon the open source work provided by the Game Boy `MiSTer` project. When checking out this repository, make sure to run the following command as this repository submodules the Game Boy `MiSTer` project.

```bash
git submodule update --init --recursive
```

### Gowin Development Environment

**The Gowin FPGA Designer v1.9.9.03 must be used.** Using Gowin IDE v1.9.10.X or newer is currently not supported by this build.

You will also need to apply for a local license with Gowin through their website:
https://www.gowinsemi.com/en/support/license

The license expires after one year and will require reactivation.

You will receive an email within a few minutes with a `.lic` file attached. Run the Gowin IDE and install the license when it prompts you. You'll need to close and re-open the GOWIN IDE if everything was successful.

## Building
Once in the IDE, load `evt1_x2.gprj` project and click on the green recycle-like button icon to run synthesis and PnR. This will take about 5-10 minutes to complete.

## Flashing
Flashing can be performed using the official [Gowin Programmer](https://www.gowinsemi.com/en/) software or the [`openFPGALoader`](https://github.com/trabucayre/openFPGALoader) utility through the Chromatic's USB interface. The Gowin Programmer requires the installation of the GWU2X device driver.

Note:
1. The Chromatic must be powered on for either tool to detect the FPGA. This means the power switch is in the **ON** position.
2. If using `openFPGALoader`, the tool must be compiled with support for the Gowin GWU2X cable.

### Example Using `openFPGALoader`
**Detect the Chromatic FPGA While Powered On**
```bash
openFPGALoader --detect --cable gwu2x
```

You will see an output similar to:
```
empty
User requested: 6000000 real frequency is 6000000
index 0:
        idcode 0x1281b
        manufacturer Gowin
        family GW5A
        model  GW5A-25
        irlength 8
```

**Flashing the Chromatic**

```bash
openFPGALoader --write-flash --cable gwu2x --reset <file>
```

Here, `<file>` refers to the generated bitstream file. This file can be found at `esp32t/impl/pnr/evt1_x2.fs`.

## Custom Modifications

When modifying the RTL design, please also update the 14-bit FPGA version within [esp32t/src/rtl/BSP/system_monitor.sv] around line 384 (see `version`).

This will ensure you can always using the [ModRetro Update Tool](https://modretro.com/pages/downloads#mrupdater) to restore your Chromatic to the latest official release.

## Issues
Please submit all issues and bug reports through our [Contact Form](https://modretro.com/pages/contact).

## Attributions
- [GOWIN Semiconductor](https://www.gowinsemi.com/en/)
- [MiSTer](https://github.com/MiSTer-devel/Gameboy_MiSTer)

## Special Thanks
- [rayjt9] For their palette improvements to the BootROM.
