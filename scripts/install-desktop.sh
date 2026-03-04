#!/bin/bash
# install-desktop.sh — Install Motor Studio as a desktop application
#
# Handles two environments:
#   A) Inside a distrobox container → creates .desktop with distrobox-enter wrapper
#   B) Native Linux (e.g. RPi5) → creates .desktop pointing directly to launcher
#
# Run from inside the workspace:
#   ./scripts/install-desktop.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"
ICON="$WORKSPACE/myactuator_python_driver/myactuator_python_driver/studio/resources/motor_studio.svg"
LAUNCHER="$SCRIPT_DIR/motor-studio.sh"
CAN_HELPER="$SCRIPT_DIR/setup_can.sh"

echo "=== Motor Studio Desktop Installer ==="
echo ""
echo "  Workspace: $WORKSPACE"

# Make scripts executable
chmod +x "$LAUNCHER" "$CAN_HELPER"

# --- Detect environment ---
IN_CONTAINER=false
CONTAINER_NAME=""
if [ -f /run/.containerenv ]; then
    IN_CONTAINER=true
    CONTAINER_NAME=$(grep '^name=' /run/.containerenv 2>/dev/null | cut -d'"' -f2)
    echo "  Environment: distrobox ($CONTAINER_NAME)"
else
    echo "  Environment: native"
fi
echo ""

# --- Build the Exec= line ---
# CAN setup runs on the HOST (needs real root for /dev/ttyACM* and ip link).
# The motor-studio.sh script will see can0 already up and skip its own CAN step.
if $IN_CONTAINER; then
    EXEC_LINE="bash -c \"sudo $CAN_HELPER can0 1000000 || true; distrobox-enter $CONTAINER_NAME -- $LAUNCHER\""
else
    EXEC_LINE="$LAUNCHER"
fi

# --- Passwordless CAN setup via sudoers ---
echo "[1/3] Configuring passwordless CAN setup..."
SUDOERS_FILE="/etc/sudoers.d/motor-studio-can"
SUDOERS_LINE="$USER ALL=(ALL) NOPASSWD: $CAN_HELPER"
if $IN_CONTAINER; then
    # Install sudoers on the HOST (where sudo actually runs for CAN setup)
    distrobox-host-exec bash -c "
        if ! sudo grep -qF '$CAN_HELPER' '$SUDOERS_FILE' 2>/dev/null; then
            echo '$SUDOERS_LINE' | sudo tee '$SUDOERS_FILE' > /dev/null
            sudo chmod 440 '$SUDOERS_FILE'
        fi
    " 2>/dev/null && echo "  Host sudoers configured." || echo "  WARNING: Could not configure host sudoers."
    # Also configure inside container (for manual runs)
    if ! sudo grep -qF "$CAN_HELPER" "$SUDOERS_FILE" 2>/dev/null; then
        echo "$SUDOERS_LINE" | sudo tee "$SUDOERS_FILE" > /dev/null
        sudo chmod 440 "$SUDOERS_FILE"
    fi
    echo "  Container sudoers configured."
else
    if ! sudo grep -qF "$CAN_HELPER" "$SUDOERS_FILE" 2>/dev/null; then
        echo "$SUDOERS_LINE" | sudo tee "$SUDOERS_FILE" > /dev/null
        sudo chmod 440 "$SUDOERS_FILE"
        echo "  Done."
    else
        echo "  Already configured."
    fi
fi

# --- Create .desktop file ---
echo "[2/3] Creating desktop entry..."

DESKTOP_CONTENT="[Desktop Entry]
Version=1.0
Type=Application
Name=Motor Studio
GenericName=Motor Control
Comment=Control and record MyActuator RMD motors
Exec=$EXEC_LINE
Icon=$ICON
Terminal=false
Categories=Development;Engineering;
StartupNotify=true
Keywords=motor;actuator;can;robotics;"

# App menu
APPS_DIR="$HOME/.local/share/applications"
mkdir -p "$APPS_DIR"
echo "$DESKTOP_CONTENT" > "$APPS_DIR/motor-studio.desktop"
update-desktop-database "$APPS_DIR" 2>/dev/null || true
echo "  Added to app menu."

# --- Desktop shortcut ---
echo "[3/3] Creating desktop shortcut..."
if [ -d "$HOME/Desktop" ]; then
    DESKTOP_FILE="$HOME/Desktop/motor-studio.desktop"
    echo "$DESKTOP_CONTENT" > "$DESKTOP_FILE"
    chmod +x "$DESKTOP_FILE"
    # Mark as trusted for KDE/GNOME
    gio set "$DESKTOP_FILE" metadata::trusted true 2>/dev/null || true
    echo "  Created: $DESKTOP_FILE"
else
    echo "  No ~/Desktop directory, skipping."
fi

echo ""
echo "=== Installation complete ==="
echo ""
echo "Launch Motor Studio by:"
echo "  - Double-clicking 'Motor Studio' on your desktop"
echo "  - Searching 'Motor Studio' in the app menu"
if $IN_CONTAINER; then
    echo "  - Running: distrobox-enter $CONTAINER_NAME -- $LAUNCHER"
else
    echo "  - Running: $LAUNCHER"
fi
echo ""
echo "Logs: ~/.local/share/motor-studio/logs/"
echo ""
