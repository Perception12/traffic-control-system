# Traffic Control System

A cloud-ready, multi-lane traffic control system built on ESP32 using PlatformIO and Arduino framework. This project demonstrates a smart traffic light controller with IR-based vehicle detection and real-time logging, suitable for educational, prototyping, or smart city applications.

## Features

- **Multi-lane Traffic Light Control:** Supports 4 independent lanes, each with its own set of red, yellow, and green lights.
- **IR Vehicle Detection:** Uses IR sensors to detect vehicles at each lane.
- **FreeRTOS Multitasking:** Runs traffic control and logging tasks concurrently on separate ESP32 cores.
- **Violation Logging:** Detects and logs vehicles crossing during red or red-yellow states.
- **Serial/Cloud Logging:** Logs sensor and traffic light status to Serial (can be extended to cloud platforms).
- **Highly Configurable:** Easily adjust timings and pin assignments for different hardware setups.

## Hardware Requirements

- ESP32 development board
- 4 IR sensors (for vehicle detection)
- 4 sets of traffic lights (each with red, yellow, green LEDs or modules)
- Resistors, breadboard, jumper wires

## Pin Configuration

| Lane | Red Pin | Yellow Pin | Green Pin | IR Sensor Pin |
|------|---------|------------|-----------|--------------|
| 1    | 19      | 18         | 5         | 34           |
| 2    | 17      | 16         | 4         | 35           |
| 3    | 27      | 26         | 25        | 22           |
| 4    | 13      | 15         | 14        | 23           |

## Software Structure

- **main.cpp:** Core logic for traffic light sequencing, IR detection, and logging.
- **FreeRTOS Tasks:**  
  - `trafficControlTask`: Manages light states and transitions.
  - `cloudLoggingTask`: Reads IR sensors and logs events.
- **PlatformIO:** Easy build and upload process.

## How It Works

1. **Initialization:** All pins are set up for sensors and lights. Serial communication is started.
2. **Traffic Control:** Each lane gets a green light in turn, with configurable durations for each state (red, red-yellow, green, yellow).
3. **Vehicle Detection:** IR sensors monitor each lane. If a vehicle is detected during a red or red-yellow state, a violation is logged.
4. **Logging:** System status and violations are printed to the Serial monitor for monitoring or further cloud integration.

## Getting Started

1. **Clone the repository:**

   ```sh
   git clone git@github.com:Perception12/traffic-control-system.git
   ```

2. **Open in PlatformIO:** Import the project folder into PlatformIO IDE.
3. **Connect ESP32:** Use a USB cable to connect your ESP32 board to the computer.
4. **Select the Right Environment:** In PlatformIO, select the appropriate environment for your ESP32 board.
5. **Build the Project:** Click on the build button (checkmark icon) in PlatformIO to compile the code.
6. **Upload to ESP32:** Click on the upload button (arrow icon) to flash the firmware to the ESP32.
7. **Open Serial Monitor:** Use the PlatformIO Serial Monitor to view logs and debug information.

## Customization

- **Timing Adjustments:** Modify the timing parameters in the code to change the duration of red, yellow, and green lights.
- **Pin Remapping:** Change the pin assignments in the configuration section of the code to match your hardware setup.
- **Sensor Calibration:** Adjust the IR sensor thresholds and detection logic as needed for your specific sensors and environment.

## Troubleshooting

- **Compilation Errors:** Ensure all libraries are installed and the correct board is selected in PlatformIO.
- **Upload Issues:** Check USB connection, cable, and ensure the correct COM port is selected.
- **Sensor Detection Problems:** Verify IR sensor connections, orientation, and adjust sensitivity in the code if necessary.

## Future Work

- **Cloud Integration:** Extend logging to cloud platforms for remote monitoring and analytics.
- **Mobile App Control:** Develop a mobile app interface for real-time control and monitoring.
- **Advanced Vehicle Detection:** Implement machine learning algorithms for more accurate vehicle detection and classification.

## Acknowledgments

- Inspired by various open-source traffic control projects.
- Built using ESP32, a powerful and versatile IoT development board.
- Utilizes PlatformIO, an open-source ecosystem for IoT development.
