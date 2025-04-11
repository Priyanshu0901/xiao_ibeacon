#!/bin/bash

# Create a virtual environment for the BLE Beacon Scanner
# This script creates a Python virtual environment and installs 
# the required packages

# Exit on error
set -e

# Set paths
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
VENV_DIR="${SCRIPT_DIR}/venv"

echo "Setting up virtual environment for BLE Beacon Scanner..."

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo "Python 3 is required but not found. Please install Python 3."
    exit 1
fi

# Create virtual environment if it doesn't exist
if [ ! -d "${VENV_DIR}" ]; then
    echo "Creating virtual environment in ${VENV_DIR}..."
    python3 -m venv "${VENV_DIR}"
else
    echo "Virtual environment already exists at ${VENV_DIR}"
fi

# Activate virtual environment
echo "Activating virtual environment..."
source "${VENV_DIR}/bin/activate"

# Install required packages
echo "Installing required packages..."
pip install --upgrade pip
pip install bleak colorama

echo "Virtual environment setup complete!"
echo ""
echo "To activate the virtual environment, run:"
echo "  source ${VENV_DIR}/bin/activate"
echo ""
echo "To run the BLE beacon scanner, run:"
echo "  python ${SCRIPT_DIR}/ble_beacon_scanner.py"
echo "" 