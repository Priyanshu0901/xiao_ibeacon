#!/usr/bin/env python3
"""
BLE Beacon Scanner for Volcatec Temperature Sensor

This script scans for BLE advertisements from the Volcatec_TempSensor beacon
and decodes the temperature, humidity, and pressure values from the 
manufacturer-specific data.

Requirements:
- bleak
- colorama

Usage:
    python ble_beacon_scanner.py
"""

import asyncio
import struct
import time
import os
import sys
from datetime import datetime
try:
    from bleak import BleakScanner
    from colorama import Fore, Style, init
except ImportError:
    print("Required packages not found. Installing them...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "bleak", "colorama"])
    from bleak import BleakScanner
    from colorama import Fore, Style, init

# Initialize colorama
init()

# Constants
ESPRESSIF_COMPANY_ID = 0x02E5
DEVICE_NAME_FILTER = "Volcatec_TempSensor"
SCAN_DURATION = 5.0  # seconds - shorter time for faster response
CLEAR_SCREEN_CMD = 'cls' if os.name == 'nt' else 'clear'
DEBUG_MODE = True  # Set to True to see debug information

# Global variables to store the latest values
latest_temp = None
latest_humidity = None
latest_pressure = None
latest_rssi = None
latest_update = None


def clear_screen():
    """Clear the terminal screen."""
    os.system(CLEAR_SCREEN_CMD)


def parse_manufacturer_data(data, company_id):
    """
    Parse manufacturer-specific data from the BLE advertisement.
    
    Format (6 bytes):
    - Byte 0: Data type (0x01)
    - Bytes 1-2: Temperature (int16, scale by 100)
    - Byte 3: Humidity (uint8, direct percentage)
    - Bytes 4-5: Pressure (uint16, scale by 10 - convert back by multiplying by 10)
    """
    if len(data) < 6:
        if DEBUG_MODE:
            print(f"Data too short: {len(data)} bytes, expected at least 6 bytes")
            print(f"Raw data: {' '.join([f'{b:02X}' for b in data])}")
        return None, None, None
    
    if DEBUG_MODE:
        print(f"Company ID: 0x{company_id:04X}")
        print(f"Raw data: {' '.join([f'{b:02X}' for b in data])}")
    
    # Parse the 6-byte format from debug output
    try:
        # First byte should be data type 0x01
        data_type = data[0]
        if data_type != 0x01:
            if DEBUG_MODE:
                print(f"Unexpected data type: 0x{data_type:02X}, expected 0x01")
            return None, None, None
        
        # Temperature (int16, scale by 100)
        temperature = struct.unpack("<h", data[1:3])[0] / 100.0
        
        # Humidity (uint8, direct percentage)
        humidity = data[3]
        
        # Pressure (uint16, multiply by 10 to get actual hPa)
        pressure_raw = (data[4] << 8) | data[5]
        pressure = pressure_raw / 10.0
        
        if DEBUG_MODE:
            print(f"Parsed values: T={temperature:.2f}°C, H={humidity}%, P={pressure:.1f} hPa")
        
        return temperature, humidity, pressure
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error parsing data: {e}")
        return None, None, None


def display_results():
    """Display the latest sensor values with a nice UI."""
    if not DEBUG_MODE:
        clear_screen()
    print(f"{Fore.CYAN}╔═══════════════════════════════════════════════╗{Style.RESET_ALL}")
    print(f"{Fore.CYAN}║ {Fore.WHITE}Volcatec Temperature Sensor Beacon Scanner{Fore.CYAN}    ║{Style.RESET_ALL}")
    print(f"{Fore.CYAN}╠═══════════════════════════════════════════════╣{Style.RESET_ALL}")
    
    if latest_update:
        print(f"{Fore.CYAN}║ {Fore.WHITE}Last Update: {latest_update.strftime('%H:%M:%S')}{Fore.CYAN}{' ' * 21}║{Style.RESET_ALL}")
        print(f"{Fore.CYAN}║ {Fore.WHITE}Signal Strength: {latest_rssi} dBm{Fore.CYAN}{' ' * 20}║{Style.RESET_ALL}")
        print(f"{Fore.CYAN}╠═══════════════════════════════════════════════╣{Style.RESET_ALL}")
        
        if latest_temp is not None:
            print(f"{Fore.CYAN}║ {Fore.YELLOW}Temperature: {Fore.GREEN}{latest_temp:.2f} °C{Fore.CYAN}{' ' * 21}║{Style.RESET_ALL}")
        else:
            print(f"{Fore.CYAN}║ {Fore.YELLOW}Temperature: {Fore.RED}Not available{Fore.CYAN}{' ' * 19}║{Style.RESET_ALL}")
            
        if latest_humidity is not None:
            print(f"{Fore.CYAN}║ {Fore.YELLOW}Humidity: {Fore.GREEN}{latest_humidity} %{Fore.CYAN}{' ' * 25}║{Style.RESET_ALL}")
        else:
            print(f"{Fore.CYAN}║ {Fore.YELLOW}Humidity: {Fore.RED}Not available{Fore.CYAN}{' ' * 22}║{Style.RESET_ALL}")
            
        if latest_pressure is not None:
            print(f"{Fore.CYAN}║ {Fore.YELLOW}Pressure: {Fore.GREEN}{latest_pressure:.1f} hPa{Fore.CYAN}{' ' * 21}║{Style.RESET_ALL}")
        else:
            print(f"{Fore.CYAN}║ {Fore.YELLOW}Pressure: {Fore.RED}Not available{Fore.CYAN}{' ' * 22}║{Style.RESET_ALL}")
    else:
        print(f"{Fore.CYAN}║ {Fore.RED}Waiting for beacon data...{Fore.CYAN}{' ' * 19}║{Style.RESET_ALL}")
        
    print(f"{Fore.CYAN}╠═══════════════════════════════════════════════╣{Style.RESET_ALL}")
    print(f"{Fore.CYAN}║ {Fore.WHITE}Press Ctrl+C to exit{Fore.CYAN}{' ' * 25}║{Style.RESET_ALL}")
    print(f"{Fore.CYAN}╚═══════════════════════════════════════════════╝{Style.RESET_ALL}")


async def scan_for_beacons():
    """Scan for BLE beacons and update values."""
    global latest_temp, latest_humidity, latest_pressure, latest_rssi, latest_update
    
    print(f"Scanning for {DEVICE_NAME_FILTER} beacons...")
    
    while True:
        try:
            devices = await BleakScanner.discover(timeout=SCAN_DURATION)
            found_device = False
            
            for device in devices:
                # Check if device name matches our filter
                if DEBUG_MODE:
                    print(f"Found device: {device.name} ({device.address}), RSSI: {device.rssi}")
                
                if device.name and DEVICE_NAME_FILTER in device.name:
                    found_device = True
                    latest_rssi = device.rssi
                    latest_update = datetime.now()
                    
                    if DEBUG_MODE:
                        print(f"Found target device: {device.name} with {len(device.metadata.get('manufacturer_data', {}))} manufacturer data entries")
                    
                    # Look for manufacturer data
                    manufacturer_data = device.metadata.get("manufacturer_data", {})
                    if DEBUG_MODE:
                        print(f"Manufacturer data entries: {len(manufacturer_data)}")
                        
                    # First try exact match for Espressif company ID
                    if ESPRESSIF_COMPANY_ID in manufacturer_data:
                        data = manufacturer_data[ESPRESSIF_COMPANY_ID]
                        temp, humidity, pressure = parse_manufacturer_data(data, ESPRESSIF_COMPANY_ID)
                        if temp is not None:
                            latest_temp = temp
                            latest_humidity = humidity
                            latest_pressure = pressure
                    else:
                        # If no exact match, try all manufacturer data blocks
                        for company_id, data in manufacturer_data.items():
                            if DEBUG_MODE:
                                print(f"Trying company ID: 0x{company_id:04X}")
                            temp, humidity, pressure = parse_manufacturer_data(data, company_id)
                            if temp is not None:
                                latest_temp = temp
                                latest_humidity = humidity
                                latest_pressure = pressure
                                break
                                
            display_results()
            
            if not found_device and DEBUG_MODE:
                print(f"No {DEVICE_NAME_FILTER} beacons found. Scanning again...")
                
            # Short pause before next scan
            await asyncio.sleep(1)
            
        except Exception as e:
            print(f"Error during scanning: {e}")
            await asyncio.sleep(5)


async def main():
    """Main function."""
    try:
        if DEBUG_MODE:
            print("Running in DEBUG mode - verbose output enabled")
        await scan_for_beacons()
    except KeyboardInterrupt:
        print("\nScanning stopped by user")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...") 