#!/bin/bash
# setup_can.sh — Bring up a CAN interface (runs as root via pkexec)
#
# Usage: setup_can.sh [interface] [bitrate]
#   interface: CAN interface name (default: can0)
#   bitrate:   CAN bitrate in bps (default: 1000000)
#
# Supports both native socketcan (gs_usb/candleLight) and SLCAN adapters.

set -euo pipefail

IFNAME="${1:-can0}"
BITRATE="${2:-1000000}"

# Map bitrate to slcand code
bitrate_to_slcan_code() {
    case "$1" in
        1000000) echo 8 ;;
        500000)  echo 6 ;;
        250000)  echo 5 ;;
        125000)  echo 4 ;;
        *)       echo 8 ;;
    esac
}

# Already up and healthy?
if ip link show "$IFNAME" up >/dev/null 2>&1; then
    echo "OK:$IFNAME already up"
    exit 0
fi

# --- Try native socketcan first (gs_usb / candleLight firmware) ---
# Look for existing CAN-capable network devices that aren't configured yet
for dev in /sys/class/net/*/type; do
    devname=$(basename "$(dirname "$dev")")
    devtype=$(cat "$dev" 2>/dev/null)
    # type 280 = ARPHRD_CAN
    if [ "$devtype" = "280" ] && [ "$devname" != "$IFNAME" ]; then
        # Found a native CAN device, rename or use it
        ip link set "$devname" down 2>/dev/null || true
        ip link set "$devname" type can bitrate "$BITRATE"
        ip link set "$devname" up
        ip link set "$devname" txqueuelen 1000
        echo "OK:Native CAN device $devname configured at ${BITRATE}bps"
        exit 0
    fi
    if [ "$devtype" = "280" ] && [ "$devname" = "$IFNAME" ]; then
        # Interface exists but is down — bring it up
        ip link set "$IFNAME" down 2>/dev/null || true
        ip link set "$IFNAME" type can bitrate "$BITRATE" 2>/dev/null || true
        ip link set "$IFNAME" up
        ip link set "$IFNAME" txqueuelen 1000
        echo "OK:$IFNAME configured at ${BITRATE}bps"
        exit 0
    fi
done

# --- Try SLCAN (serial USB-CAN adapter) ---
SERIAL_DEV=""
for dev in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$dev" ] && SERIAL_DEV="$dev" && break
done

if [ -z "$SERIAL_DEV" ]; then
    echo "FAIL:No CAN adapter found. Connect a USB-CAN adapter and try again."
    exit 1
fi

# Kill any existing slcand on this device
pkill -f "slcand.*$SERIAL_DEV" 2>/dev/null || true
sleep 0.3

# Remove stale interface
ip link delete "$IFNAME" 2>/dev/null || true
sleep 0.2

SLCAN_CODE=$(bitrate_to_slcan_code "$BITRATE")
slcand -o -c -s"$SLCAN_CODE" "$SERIAL_DEV" "$IFNAME"
sleep 0.5

ip link set up "$IFNAME"
ip link set "$IFNAME" txqueuelen 1000

echo "OK:SLCAN on $SERIAL_DEV → $IFNAME at ${BITRATE}bps"
exit 0
