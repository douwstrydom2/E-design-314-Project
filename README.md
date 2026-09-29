# E-DAS: Embedded Driver Assistance System

An STM32-based embedded system that monitors vehicle and environmental conditions and warns the driver in real time when unsafe conditions are detected. Built for **Design (E) 314** at Stellenbosch University (2026). **Final mark: 92%.**

![E-DAS board](docs/board.jpg)
<!-- Add a photo of the finished board here -->

## Features

- **Obstacle detection** with an HC-SR04 ultrasonic sensor (2–400 cm range, within 2 cm of actual distance in testing)
- **Motion and impact monitoring** with an MPU-6050 accelerometer (unsafe-driving and impact warnings)
- **Ambient light sensing** with an SFH203 photodiode and op-amp transimpedance amplifier
- **Temperature monitoring** with an LMT01 pulse-count sensor and comparator circuit
- **OLED menu interface** driven by a finite-state machine, navigated with push buttons and a 4×3 keypad
- **SD card logging** of measurements and warning events to CSV (periodic and event-triggered)
- **UART command interface** for telemetry, warning overrides, logging control and clock setting
- **Status LEDs** for proximity, impact/unsafe driving, low light and high temperature

## Hardware

| Component | Interface |
|---|---|
| STM32F411RE (Nucleo-64) | Main controller |
| HC-SR04 ultrasonic sensor | GPIO trigger + TIM2 input capture |
| MPU-6050 accelerometer | I2C (0x68) |
| SSD1306 128×32 OLED | I2C (0x3C) |
| LMT01 temperature sensor | EXTI pulse counting via comparator |
| SFH203 photodiode | ADC1 channel 9 via transimpedance amplifier |
| SD card module | SPI1 + FatFs |
| 4×3 keypad, 5 push buttons | GPIO / EXTI with software debouncing |

Power: 9 V input, regulated to 5 V (L7805) and 3.3 V (LD1117).

## Software design

A single non-blocking main loop schedules each task on its own interval using `HAL_GetTick()`, so sensor polling, display updates, logging and UART handling don't block each other. Key modules:

- `HCSR04_Read()`: echo pulse timing with timer input capture
- `MPU6050_Read_All()`: 6-byte burst read, gravity-compensated net acceleration
- `Update_Alarm_Conditions()`: threshold checks, LED flags and warning state
- `Handle_Button_Press()`: menu finite-state machine
- `SD_Log_Data()`: CSV logging with file closed after every write to prevent data loss
- `Process_UART_Command()`: command parsing

## UART commands

57600 baud, 9-bit word, even parity. Commands use the `@...&` frame format:

| Command | Action |
|---|---|
| `@Stat&` | Return live measurements and warning status |
| `@SetWarn&` | Override individual warning channels |
| `@Log&` / `@Dump&` / `@CLF&` | SD card logging controls |
| `@SetRTC YYYY/MM/DD;HH:MM:SS&` | Set the system clock |
| `@SFD&` / `@RFE&` | Fuel data commands |

## Build and run

1. Open the project in **STM32CubeIDE**.
2. Build and flash to an STM32F411RE Nucleo board.
3. Connect a serial terminal at 57600 baud (9-bit, even parity) to use the command interface.

## Documentation

The full technical report, including circuit diagrams, test measurements and oscilloscope results, is in [`docs/E-DAS_Report.pdf`](docs/E-DAS_Report.pdf).

## Possible improvements

- Custom PCB to replace point-to-point wiring
- Lower-power operating modes and reduced bus polling
- Non-linear sensor calibration for better accuracy

## Credits

OLED graphics library: [stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306) by Aleksander Alekseev.

## Author

Douw (Gideon Louwrens) Strydom · [LinkedIn](https://www.linkedin.com/in/douw-strydom-72a741286)
