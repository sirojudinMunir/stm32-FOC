# Field-Oriented Control (FOC) on STM32 From Scratch

This project demonstrates a complete, low-level implementation of **Field-Oriented Control (FOC)** for controlling a **BLDC motor** using an **STM32F4 microcontroller**, without relying on high-level motor control libraries.

Built using **PlatformIO**, **STM32 HAL/LL drivers**, and **FreeRTOS**, this project is designed for learning and practical experimentation in precision motor control.

---

## Features Implemented

- Clarke & Park Transforms  
- Position feedback via AS5047 encoder  
- Phase current sampling via ADC  
- SVPWM (Space Vector PWM) generation  
- Torque Control
- Speed Control
- Position Control
- Encoder auto callibration
- Self-Commissioning for PI Gain current controller auto tuning
- Sensorless mode using Sliding Mode Observer (SMO) & High Frequency Injection (HFI)
- USB CDC interface for real-time tuning and monitoring  
- Save configuration to flash memory
- Interrupt-driven timing and FreeRTOS tasking  

---

## Development Tools

- [PlatformIO](https://platformio.org/)
- STM32CubeMX
- STM32Cube HAL & LL Drivers  
- FreeRTOS (via PlatformIO package)  
- VS Code + Serial Monitor for debugging
- Altium Designer (for Hardware) you can see in STM32-FOC-hardware.zip
- ACTUATOR CONTROL APP in controller_app.zip

---

## UPDATE LOG

03-02-2026
- Add Initial position detection with polarity judgement

20-01-2026
- Update HFI using single-bin Sliding DFT

26-11-2025
- Add sensorless mode using High Frequency Injection (HFI)
- Hysteresis switching between SMO and HFI

11-11-2025
- Add sensorless mode using Sliding Mode Observer (SMO)
- Dynamic max output PI controller Id Iq based on Vbus
- Save Rs, Ld, Lq to flash memory
- Move parse CLI code to controller_app

30-10-2025
- change algorithm to estimate resistance using constant voltage (vd)

15-10-2025
- self commissioning, auto calculate Motor Resistance and Inductance (Rs, Ld, and Lq)
- Auto Calculate PI Gain Current Control loop using R, L, and Bandwidth
- Setting bandwidth using CLI
  set_bandwidth=(bandwidth in Hz)
- move custom library code to lib folder

6-10-2025
- CAN bus
- Haptic control demo
- Setting Error Deadband and Max out PID Controler using CLI

16-9-2025
- Update audio mode

20-8-2025
- ADC injected simultaneous mode to read currents and DC voltage faster
- Add command to change title, legends, and erase graph in serial plotter

9-8-2025
- Add eccentricity calibration for magnetic encoder
- CLI for auto calibration
- Change PID manual tuning settings
- Save configuration to flash memory
- Default config

29-7-2025
- Add parity check for AS5047P

---