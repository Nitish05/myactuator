#!/bin/bash
# motor-studio.sh — Launch Motor Studio with automatic CAN and ROS setup
#
# Works in three environments:
#   1. Inside a distrobox container (desktop launches via distrobox-export)
#   2. Native Linux with ROS 2 installed (e.g. RPi5 with Jazzy)
#   3. Direct invocation from terminal
#
# The script auto-detects the ROS distro and workspace location.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Configuration ---
WORKSPACE="${MOTOR_STUDIO_WORKSPACE:-$(dirname "$SCRIPT_DIR")}"
CAN_INTERFACE="${MOTOR_STUDIO_CAN:-can0}"
CAN_BITRATE="${MOTOR_STUDIO_BITRATE:-1000000}"
LOG_DIR="$HOME/.local/share/motor-studio/logs"
ICON="$WORKSPACE/myactuator_python_driver/myactuator_python_driver/studio/resources/motor_studio.svg"

# --- Helpers ---
timestamp() { date '+%Y-%m-%d %H:%M:%S'; }

notify() {
    notify-send -i "$ICON" "Motor Studio" "$1" 2>/dev/null || true
}

die() {
    echo "[$(timestamp)] FATAL: $1" >&2
    notify "$1"
    # Try graphical error dialog
    if command -v kdialog >/dev/null 2>&1; then
        kdialog --error "$1" --title "Motor Studio" 2>/dev/null
    elif command -v zenity >/dev/null 2>&1; then
        zenity --error --text="$1" --title="Motor Studio" 2>/dev/null
    fi
    exit 1
}

cleanup() {
    echo "[$(timestamp)] Shutting down..."
    if [ -n "${DRIVER_PID:-}" ] && kill -0 "$DRIVER_PID" 2>/dev/null; then
        kill "$DRIVER_PID" 2>/dev/null
        wait "$DRIVER_PID" 2>/dev/null || true
    fi
    sleep 0.3
    echo "[$(timestamp)] Done."
}

trap cleanup EXIT INT TERM

# --- Logging ---
mkdir -p "$LOG_DIR"
LOGFILE="$LOG_DIR/$(date '+%Y%m%d_%H%M%S').log"
ls -t "$LOG_DIR"/*.log 2>/dev/null | tail -n +11 | xargs rm -f 2>/dev/null || true

exec > >(tee -a "$LOGFILE") 2>&1
echo "[$(timestamp)] Motor Studio starting..."
echo "[$(timestamp)] Log: $LOGFILE"

# Detect environment
if [ -f /run/.containerenv ]; then
    CONTAINER_NAME=$(grep '^name=' /run/.containerenv 2>/dev/null | cut -d'"' -f2)
    echo "[$(timestamp)] Running inside distrobox container: ${CONTAINER_NAME:-unknown}"
fi

# --- Step 1: CAN Bus Setup ---
echo "[$(timestamp)] Setting up CAN ($CAN_INTERFACE @ ${CAN_BITRATE}bps)..."

CAN_OK=false
if ip link show "$CAN_INTERFACE" up >/dev/null 2>&1; then
    echo "[$(timestamp)] $CAN_INTERFACE already up."
    CAN_OK=true
else
    CAN_HELPER="$SCRIPT_DIR/setup_can.sh"
    if [ -x "$CAN_HELPER" ]; then
        RESULT=$(sudo "$CAN_HELPER" "$CAN_INTERFACE" "$CAN_BITRATE" 2>&1) || true
        echo "[$(timestamp)] CAN setup: $RESULT"

        if [[ "$RESULT" == OK:* ]]; then
            CAN_OK=true
        fi
    else
        echo "[$(timestamp)] CAN helper not found: $CAN_HELPER"
    fi
fi

if $CAN_OK; then
    echo "[$(timestamp)] CAN OK."
else
    echo "[$(timestamp)] WARNING: CAN not available — studio will open without hardware."
    notify "No CAN adapter — opening without hardware"
fi

# --- Step 2: Source ROS 2 (auto-detect distro) ---
echo "[$(timestamp)] Sourcing ROS 2..."

ROS_SETUP=""
for distro_dir in /opt/ros/*/; do
    candidate="${distro_dir}setup.bash"
    if [ -f "$candidate" ]; then
        ROS_SETUP="$candidate"
    fi
done

if [ -z "$ROS_SETUP" ]; then
    die "No ROS 2 installation found in /opt/ros/"
fi

# ROS setup scripts use unbound variables — temporarily allow that
set +u
source "$ROS_SETUP"

# Source workspace
WS_SETUP="$WORKSPACE/install/setup.bash"
if [ ! -f "$WS_SETUP" ]; then
    set -u
    die "Workspace not built.

Run:
  cd $WORKSPACE
  colcon build --cmake-args -DPYTHON_BINDINGS=on"
fi
source "$WS_SETUP"
set -u

echo "[$(timestamp)] ROS 2 $ROS_DISTRO + workspace sourced."

# --- Step 3: Start driver node (only if CAN is available) ---
if $CAN_OK; then
    echo "[$(timestamp)] Starting motor driver..."
    ros2 run myactuator_python_driver driver_node &
    DRIVER_PID=$!
    echo "[$(timestamp)] Driver PID: $DRIVER_PID"

    sleep 1.5

    if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
        echo "[$(timestamp)] WARNING: Driver failed to start. Opening studio anyway."
        notify "Driver failed — opening studio without hardware"
        DRIVER_PID=""
    else
        echo "[$(timestamp)] Driver running."
        notify "Connected and ready"
    fi
else
    echo "[$(timestamp)] Skipping driver (no CAN)."
    DRIVER_PID=""
fi

# --- Step 4: Launch Motor Studio GUI ---
echo "[$(timestamp)] Opening Motor Studio..."
ros2 run myactuator_python_driver motor_studio

echo "[$(timestamp)] Motor Studio closed."
