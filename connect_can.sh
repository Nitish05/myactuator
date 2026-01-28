#!/bin/bash

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check for slcand
if ! command_exists slcand; then
    echo "Error: 'slcand' not found. Please install can-utils:"
    echo "  sudo apt install can-utils"
    exit 1
fi

echo "Scanning for serial devices..."
serial_devices=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null)

if [ -z "$serial_devices" ]; then
    echo "No serial devices found (/dev/ttyUSB* or /dev/ttyACM*)."
    exit 1
fi

# List devices
i=1
declare -a devices
echo "Available devices:"
for dev in $serial_devices; do
    # Try to get more info from udev if possible, or just ls -l
    echo "  [$i] $dev"
    devices[$i]=$dev
    ((i++))
done

# Prompt for selection
read -p "Select device number [1]: " dev_num
dev_num=${dev_num:-1}
selected_dev=${devices[$dev_num]}

if [ -z "$selected_dev" ]; then
    echo "Invalid selection."
    exit 1
fi

echo "Selected: $selected_dev"

# Prompt for interface name
read -p "Enter CAN interface name [can0]: " ifname
ifname=${ifname:-can0}

# Prompt for bitrate (slcand flag)
# -s8 = 1000000
# -s6 = 500000
# -s5 = 250000
echo "Select bitrate:"
echo "  [8] 1000 kbit/s (default)"
echo "  [6] 500 kbit/s"
echo "  [5] 250 kbit/s"
read -p "Enter bitrate code [8]: " bitrate_code
bitrate_code=${bitrate_code:-8}

# Check if interface already exists
if ip link show "$ifname" >/dev/null 2>&1; then
    echo "Interface $ifname already exists."
    read -p "Do you want to shut it down and restart it? [y/N]: " restart_choice
    if [[ "$restart_choice" =~ ^[Yy]$ ]]; then
        sudo ip link set down "$ifname"
        # Kill associated slcand process
        sudo pkill -f "slcand.*$selected_dev" 2>/dev/null
        # Wait a bit
        sleep 1
    else
        echo "Aborting."
        exit 0
    fi
fi

echo "Starting slcand..."
# -o: open command
# -c: close command (cleanup)
# -sX: set bitrate
# -F: stay in foreground (daemonize is default without -F, but we want to know if it fails)
# Actually slcand daemonizes by default.
cmd="sudo slcand -o -c -s$bitrate_code $selected_dev $ifname"
echo "Running: $cmd"
$cmd

if [ $? -ne 0 ]; then
    echo "Failed to start slcand."
    exit 1
fi

echo "Bringing up interface $ifname..."
sudo ip link set up "$ifname"
sudo ip link set "$ifname" txqueuelen 1000

if [ $? -eq 0 ]; then
    echo "Success! Interface $ifname is up."
    ip -s link show "$ifname"
else
    echo "Failed to bring up interface."
fi
