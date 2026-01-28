#!/usr/bin/env python3
"""
Terminal User Interface (TUI) for MyActuator driver setup.

Allows interactive configuration of:
- CAN interface selection
- Motor discovery and selection
- Joint naming and configuration
- Torque override rules
"""

import os
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

from myactuator_python_driver.config import (
    DriverConfig, MotorConfig, TorqueRule, 
    MOTOR_TORQUE_CONSTANTS, get_torque_constant
)
from myactuator_python_driver.can_utils import CANScanner, MotorInfo


def clear_screen():
    """Clear the terminal screen."""
    os.system('cls' if os.name == 'nt' else 'clear')


def print_header(title: str):
    """Print a formatted header."""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60 + "\n")


def print_menu(options: List[str], selected: int = 0) -> None:
    """Print a menu with options."""
    for i, opt in enumerate(options):
        prefix = "→ " if i == selected else "  "
        print(f"{prefix}{i + 1}. {opt}")


def get_input(prompt: str, default: str = "") -> str:
    """Get input with a default value."""
    if default:
        result = input(f"{prompt} [{default}]: ").strip()
        return result if result else default
    return input(f"{prompt}: ").strip()


def get_int_input(prompt: str, default: int, min_val: int = None, max_val: int = None) -> int:
    """Get integer input with validation."""
    while True:
        try:
            result = input(f"{prompt} [{default}]: ").strip()
            value = int(result) if result else default
            if min_val is not None and value < min_val:
                print(f"Value must be at least {min_val}")
                continue
            if max_val is not None and value > max_val:
                print(f"Value must be at most {max_val}")
                continue
            return value
        except ValueError:
            print("Please enter a valid integer")


def get_float_input(prompt: str, default: float) -> float:
    """Get float input with validation."""
    while True:
        try:
            result = input(f"{prompt} [{default}]: ").strip()
            return float(result) if result else default
        except ValueError:
            print("Please enter a valid number")


def get_yes_no(prompt: str, default: bool = True) -> bool:
    """Get yes/no input."""
    default_str = "Y/n" if default else "y/N"
    result = input(f"{prompt} [{default_str}]: ").strip().lower()
    if not result:
        return default
    return result in ('y', 'yes', '1', 'true')


def select_can_interface() -> str:
    """Select CAN interface interactively."""
    print_header("CAN Interface Selection")
    
    interfaces = CANScanner.get_available_interfaces()
    
    if interfaces:
        print("Detected CAN interfaces:")
        for i, iface in enumerate(interfaces):
            status = "UP" if CANScanner.is_interface_up(iface) else "DOWN"
            print(f"  {i + 1}. {iface} ({status})")
        print(f"  {len(interfaces) + 1}. Enter manually")
        print()
        
        choice = get_int_input("Select interface", 1, 1, len(interfaces) + 1)
        if choice <= len(interfaces):
            return interfaces[choice - 1]
    else:
        print("No CAN interfaces detected.")
    
    return get_input("Enter CAN interface name", "can0")


def scan_for_motors(interface: str) -> List[MotorInfo]:
    """Scan for motors on the CAN bus."""
    print_header("Motor Discovery")
    
    print(f"Scanning for motors on {interface}...")
    print("This may take a few seconds.\n")
    
    scanner = CANScanner(interface)
    
    def progress(current, total):
        bar_len = 40
        filled = int(bar_len * current / total)
        bar = "█" * filled + "░" * (bar_len - filled)
        print(f"\rScanning: [{bar}] {current}/{total}", end='', flush=True)
    
    start_id = get_int_input("Start ID", 1, 1, 32)
    end_id = get_int_input("End ID", 10, start_id, 32)
    
    print()
    motors = scanner.scan_all((start_id, end_id), progress)
    print("\n")
    
    if motors:
        print(f"Found {len(motors)} motor(s):\n")
        print(f"  {'ID':<6} {'Temp':<8} {'Voltage':<10} {'Status'}")
        print("  " + "-" * 40)
        for m in motors:
            error_str = "OK" if m.error_code == 0 else f"Error: 0x{m.error_code:04X}"
            print(f"  {m.can_id:<6} {m.temperature}°C{'':<4} {m.voltage:.1f}V{'':<5} {error_str}")
    else:
        print("No motors found.")
        print("Make sure motors are powered and CAN interface is up.")
    
    print()
    return motors


def configure_motors(discovered: List[MotorInfo]) -> List[MotorConfig]:
    """Configure motors interactively."""
    print_header("Motor Configuration")
    
    configs = []
    
    # Get number of motors
    if discovered:
        print(f"Discovered motors: {[m.can_id for m in discovered]}")
        use_discovered = get_yes_no("Use discovered motors?", True)
        if use_discovered:
            motor_ids = [m.can_id for m in discovered]
        else:
            motor_ids = []
    else:
        motor_ids = []
    
    if not motor_ids:
        num_motors = get_int_input("Number of motors to configure", 1, 1, 32)
        motor_ids = []
        for i in range(num_motors):
            can_id = get_int_input(f"CAN ID for motor {i + 1}", i + 1, 1, 32)
            motor_ids.append(can_id)
    
    print()
    
    # Configure each motor
    for i, can_id in enumerate(motor_ids):
        print(f"\n--- Motor {i + 1} (CAN ID: {can_id}) ---\n")
        
        joint_name = get_input(f"Joint name", f"joint{i + 1}")
        
        # Torque constant
        print("\nMotor models with known torque constants:")
        models = list(MOTOR_TORQUE_CONSTANTS.keys())[:8]
        print(f"  {', '.join(models)}, ...")
        
        motor_model = get_input("Motor model (or press Enter for default)", "X8V2")
        torque_constant = get_torque_constant(motor_model)
        torque_constant = get_float_input("Torque constant (Nm/A)", torque_constant)
        
        max_velocity = get_float_input("Max velocity (deg/s)", 720.0)
        inverted = get_yes_no("Invert direction?", False)
        
        # Torque override rule
        add_torque_rule = get_yes_no("Add torque override rule?", False)
        torque_rule = None
        if add_torque_rule:
            threshold = get_float_input("Angle threshold (degrees)", 45.0)
            torque = get_float_input("Override torque (Nm)", 1.0)
            direction = get_input("Direction (above/below)", "above")
            torque_rule = TorqueRule(
                threshold_deg=threshold,
                torque_nm=torque,
                direction=direction
            )
        
        configs.append(MotorConfig(
            can_id=can_id,
            joint_name=joint_name,
            torque_constant=torque_constant,
            max_velocity=max_velocity,
            inverted=inverted,
            torque_rule=torque_rule,
        ))
    
    return configs


def configure_driver_settings() -> Tuple[float, str]:
    """Configure general driver settings."""
    print_header("Driver Settings")
    
    publish_rate = get_float_input("Publish rate (Hz)", 100.0)
    
    print("\nControl modes:")
    print("  1. position - Position control (default)")
    print("  2. velocity - Velocity control")
    print("  3. torque   - Torque/effort control")
    print("  4. readonly - Read-only, no control")
    
    mode = get_input("Control mode", "position")
    if mode not in ('position', 'velocity', 'torque', 'readonly'):
        mode = 'position'
    
    return publish_rate, mode


def save_config(config: DriverConfig):
    """Save configuration to file."""
    print_header("Save Configuration")
    
    default_path = DriverConfig.default_config_path()
    print(f"Default config path: {default_path}")
    
    custom_path = get_input("Save path (or press Enter for default)", str(default_path))
    save_path = Path(custom_path)
    
    config.save(save_path)
    print(f"\n✓ Configuration saved to: {save_path}")
    
    # Also show the YAML content
    print("\n--- Configuration Preview ---\n")
    print(config.to_yaml())


def show_summary(config: DriverConfig):
    """Show configuration summary."""
    print_header("Configuration Summary")
    
    print(f"CAN Interface: {config.can_interface}")
    print(f"Publish Rate:  {config.publish_rate} Hz")
    print(f"Control Mode:  {config.control_mode}")
    print(f"Motors:        {len(config.motors)}")
    print()
    
    for m in config.motors:
        print(f"  • {m.joint_name} (ID {m.can_id})")
        print(f"    Torque constant: {m.torque_constant} Nm/A")
        print(f"    Max velocity: {m.max_velocity} deg/s")
        if m.inverted:
            print("    Direction: Inverted")
        if m.torque_rule:
            r = m.torque_rule
            print(f"    Torque rule: {r.torque_nm} Nm when {r.direction} {r.threshold_deg}°")


def run_setup_wizard():
    """Run the complete setup wizard."""
    clear_screen()
    print_header("MyActuator Python Driver Setup")
    print("This wizard will help you configure the motor driver.\n")
    
    # Step 1: Select CAN interface
    can_interface = select_can_interface()
    print(f"\n✓ Selected interface: {can_interface}\n")
    
    # Step 2: Scan for motors
    input("Press Enter to scan for motors...")
    discovered = scan_for_motors(can_interface)
    
    # Step 3: Configure motors
    input("Press Enter to continue with motor configuration...")
    motors = configure_motors(discovered)
    
    # Step 4: Driver settings
    publish_rate, control_mode = configure_driver_settings()
    
    # Create config
    config = DriverConfig(
        can_interface=can_interface,
        motors=motors,
        publish_rate=publish_rate,
        control_mode=control_mode,
    )
    
    # Show summary
    show_summary(config)
    
    # Save
    print()
    if get_yes_no("Save this configuration?", True):
        save_config(config)
    
    print("\n" + "=" * 60)
    print("  Setup complete!")
    print("=" * 60)
    print("\nTo start the driver, run:")
    print("  ros2 run myactuator_python_driver driver_node")
    print("\nOr with a launch file:")
    print("  ros2 launch myactuator_python_driver driver.launch.py")
    print()


def main():
    """Main entry point."""
    try:
        run_setup_wizard()
    except KeyboardInterrupt:
        print("\n\nSetup cancelled.")
        sys.exit(0)
    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
