# ESP32 BLE Temperature Sensor Beacon

A modular and energy-efficient firmware for an ESP32-based low-power BLE temperature sensor beacon. This project integrates a BME280 environmental sensor with BLE advertising capabilities, designed for low-power operation and reliable data transmission.

## Features

- **Environmental Sensing**
  - Temperature measurement
  - Humidity measurement
  - Pressure measurement
  - Configurable sampling intervals
  - Automatic sensor calibration

- **BLE Communication**
  - Low-power BLE advertising
  - Manufacturer-specific data format
  - Configurable advertising intervals
  - Device name broadcasting
  - Custom service UUID

- **Power Management**
  - Deep sleep mode
  - Configurable wake-up intervals
  - Automatic power state management
  - Battery level monitoring (optional)

- **Modular Architecture**
  - Component-based design
  - Configurable through Kconfig
  - Support for both I2C and SPI interfaces
  - Easy to extend and modify

## Hardware Requirements

- ESP32 development board
- BME280 environmental sensor
- Power source (battery or USB)
- Optional: Battery level monitoring circuit

## Software Requirements

- ESP-IDF v5.4 or later
- Python 3.8+ (for test scripts)
- Bluetooth adapter (for testing)

## Project Structure

```
temp-ble-beacon/
├── components/
│   ├── ble_beacon/            # BLE advertising component
│   ├── power_manager/         # Power management component
│   └── sensor_t_a_h/          # Temperature, humidity, pressure sensor
│       ├── include/           # Public headers
│       ├── priv_include/      # Private headers
│       ├── BME280_SensorAPI/  # BME280 driver
│       ├── common.c           # Common sensor functions
│       ├── sensor_api.c       # Sensor API implementation
│       ├── sensor_t_a_h.c     # Main sensor component
│       └── Kconfig            # Component configuration
├── main/
│   ├── main.c                 # Main application
│   └── CMakeLists.txt         # Main component build configuration
├── test_scripts/              # Python test utilities
└── sdkconfig                  # ESP-IDF configuration
```

## Building and Flashing

1. Set up ESP-IDF environment:
   ```bash
   . $IDF_PATH/export.sh
   ```

2. Configure the project:
   ```bash
   idf.py menuconfig
   ```

3. Build the project:
   ```bash
   idf.py build
   ```

4. Flash to device:
   ```bash
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

## Configuration Options

### Sensor Configuration
- Interface selection (I2C/SPI)
- Sampling interval
- Sensor resolution
- Calibration settings

### BLE Configuration
- Advertising interval
- Device name
- Service UUID
- Manufacturer data format

### Power Management
- Deep sleep duration
- Wake-up triggers
- Power saving modes

## Testing

### Using nRF Connect
1. Install nRF Connect app on your mobile device
2. Scan for BLE devices
3. Look for device named "Volcatec_TempSensor"
4. View manufacturer-specific data for sensor readings

### Using Python Test Script
1. Set up Python environment:
   ```bash
   cd test_scripts
   ./setup_venv.sh  # Linux/macOS
   # or
   setup_venv.bat   # Windows
   ```

2. Run the scanner:
   ```bash
   python ble_beacon_scanner.py
   ```

## Data Format

The BLE advertisement data contains:
- Company ID: 0xFFFF (Volcatec)
- Data type: 0x01 (Temperature)
- Data type: 0x02 (Humidity)
- Data type: 0x03 (Pressure)

Each measurement is encoded as a 16-bit signed integer with appropriate scaling.

## Troubleshooting

### Common Issues
1. **Sensor Not Detected**
   - Check I2C/SPI connections
   - Verify power supply
   - Check sensor address/CS pin

2. **BLE Advertising Issues**
   - Verify BLE configuration
   - Check advertising interval
   - Ensure device name is set

3. **Power Management Issues**
   - Verify deep sleep configuration
   - Check wake-up sources
   - Monitor power consumption

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ESP-IDF team for the excellent framework
- Bosch Sensortec for the BME280 sensor
- Nordic Semiconductor for nRF Connect 