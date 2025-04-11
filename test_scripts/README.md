# BLE Beacon Scanner

This is a Python tool to scan for and decode BLE advertisements from Volcatec_TempSensor beacons.

## Features

- Scans for BLE beacons matching "Volcatec_TempSensor" name
- Decodes temperature, humidity, and pressure from the manufacturer-specific data
- Displays values in a colorful user interface
- Continuously updates in real-time
- Cross-platform: works on Linux, macOS, and Windows

## Requirements

- Python 3.7 or newer
- Bluetooth adapter supporting BLE
- Proper Bluetooth permissions on your system

## Dependencies

- bleak: For Bluetooth Low Energy scanning
- colorama: For colorful terminal output

## Installation

### Linux/macOS

1. Make the setup script executable:
   ```bash
   chmod +x setup_venv.sh
   ```

2. Run the setup script:
   ```bash
   ./setup_venv.sh
   ```

3. Activate the virtual environment:
   ```bash
   source venv/bin/activate
   ```

### Windows

1. Run the setup script by double-clicking `setup_venv.bat` or running it from Command Prompt

2. Activate the virtual environment:
   ```
   venv\Scripts\activate.bat
   ```

## Usage

Once the virtual environment is activated:

```bash
python ble_beacon_scanner.py
```

The scanner will continuously look for BLE advertisements from Volcatec_TempSensor devices and display the decoded sensor values.

Press Ctrl+C to exit the scanner.

## Troubleshooting

### Bluetooth Permission Issues (Linux)

If you get permission errors on Linux, you may need to run the scanner with sudo or add your user to the bluetooth group:

```bash
sudo python ble_beacon_scanner.py
```

Or, for a more permanent solution:

```bash
sudo usermod -a -G bluetooth $USER
```

Then log out and back in.

### Bluetooth Adapter Not Found

Make sure your Bluetooth adapter is enabled and functioning. On Linux, you can check with:

```bash
hciconfig
```

### Cannot Find Beacon

- Make sure the beacon is active and advertising
- Check that the beacon name is correctly set to "Volcatec_TempSensor"
- Ensure you're within range of the beacon (typically 10-30 meters in open space)

## Data Format

The beacon data is encoded in the manufacturer-specific data field with:
- Company ID: 0x02E5 (Espressif)
- Data format:
  - Bytes 0-1: Company ID (0x02E5)
  - Byte 2: Data type (0x01)
  - Bytes 3-4: Temperature (int16, scale by 0.01 to get °C)
  - Byte 5: Humidity (uint8, direct percentage)
  - Bytes 6-7: Pressure (MSB-LSB, scale by 10 to get hPa) 