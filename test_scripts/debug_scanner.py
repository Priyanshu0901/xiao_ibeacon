#!/usr/bin/env python3
"""
Debug BLE Scanner

This script scans for all BLE advertisements and prints the raw data
to help diagnose issues with the Xiao_TempSensor beacon.
"""

import asyncio
import sys

try:
    from bleak import BleakScanner
except ImportError:
    print("Required packages not found. Installing them...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "bleak"])
    from bleak import BleakScanner

SCAN_DURATION = 10.0  # seconds

async def scan_for_devices():
    """Scan for all BLE devices and print raw data."""
    print("Scanning for all BLE devices...")
    
    devices = await BleakScanner.discover(timeout=SCAN_DURATION)
    print(f"Found {len(devices)} BLE devices")
    
    for i, device in enumerate(devices):
        print(f"\nDevice {i+1}: {device.name or 'Unknown'} ({device.address})")
        print(f"  RSSI: {device.rssi} dBm")
        
        # Print all metadata
        for key, value in device.metadata.items():
            if key == "manufacturer_data":
                print(f"  Manufacturer Data:")
                for company_id, data in value.items():
                    print(f"    Company ID: 0x{company_id:04X}")
                    print(f"    Data: {' '.join([f'{b:02X}' for b in data])}")
            else:
                print(f"  {key}: {value}")
        
        # Special handling for Xiao sensor
        if device.name and "Xiao" in device.name:
            print(f"  *** FOUND Xiao DEVICE ***")
            
            # Check for manufacturer data
            for company_id, data in device.metadata.get("manufacturer_data", {}).items():
                print(f"  Company ID: 0x{company_id:04X} (Espressif is 0x02E5)")
                print(f"  Raw Data: {' '.join([f'{b:02X}' for b in data])}")
                
                if len(data) >= 8:
                    # Try to parse temperature
                    import struct
                    temp_bytes = data[3:5]
                    temp = struct.unpack("<h", temp_bytes)[0] / 100.0
                    
                    # Humidity is just a direct byte
                    humidity = data[5]
                    
                    # Pressure is two bytes
                    pressure_msb = data[6]
                    pressure_lsb = data[7]
                    pressure = ((pressure_msb << 8) | pressure_lsb) * 10.0
                    
                    print(f"  Parsed Values:")
                    print(f"    Temperature: {temp:.2f}°C (bytes: {temp_bytes.hex()})")
                    print(f"    Humidity: {humidity}%")
                    print(f"    Pressure: {pressure:.1f} hPa")

async def main():
    """Main function."""
    try:
        await scan_for_devices()
    except KeyboardInterrupt:
        print("\nScanning stopped by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...") 