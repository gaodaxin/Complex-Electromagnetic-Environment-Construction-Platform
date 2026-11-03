# Complex Electromagnetic Environment Generation Platform

A hardware-based platform for generating configurable radar waveforms. This repository includes an eight-channel FPGA signal generation core, embedded RFSoC initialization code, and a Windows control application.

## Features

- Eight independently configurable waveform channels.
- Continuous-wave and pulsed signals, linear/nonlinear frequency modulation (LFM/NLFM), and phase-coded waveforms.
- Frequency hopping, fixed/staggered/jittered/sliding pulse repetition intervals (PRI), and antenna scanning envelope simulation.
- RS-422 serial control with parameter loading and start/stop commands.
- RTL designed for a 300 MHz waveform clock with 16 samples per cycle per channel, equivalent to 4.8 GSPS digital sample throughput.

## Repository Structure

| Directory | Contents |
| --- | --- |
| `PL/` | Verilog waveform generation and serial command logic. Top module: `signal_core_vu13p.v`. |
| `PS/` | Embedded C code for RF data converter initialization, synchronization, HMC7044 clock control, and register access. Entry point: `main.c`. |
| `RadarConsole_Windows_Portable_v1.0.1/` | Portable Windows application (`RadarConsole.exe`) and example JSON configuration. |

## Getting Started

1. **FPGA:** Add all Verilog files under `PL/` to a Vivado project targeting the intended hardware. Integrate `signal_core_vu13p` with the board's clocks, reset, RS-422 interface, and DAC data path. The core requires 100 MHz configuration and 300 MHz waveform clocks.
2. **Embedded software:** Build `PS/` in a matching Xilinx SDK/Vitis hardware platform with its BSP, RFdc driver, and libmetal dependencies. Adapt board addresses and clock settings as needed.
3. **Host control:** On Windows with a compatible .NET Framework 4.7 runtime, open `RadarConsole.exe` in the supplied portable application folder. Connect through RS-422 at 115200 baud, configure the waveform, and send the settings. Release the DDS global reset (`0x4A = 0`) before loading and starting waveforms.

## Build Notes

The PL core targets VU13P, while the PS code requires an RFSoC hardware platform. Board integration is required. Vivado project files, pin constraints, hardware exports, BSPs, bitstreams, and host application source code are not included in this package.

