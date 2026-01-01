#!/bin/bash
# Verification script for energy_meter_solver installation

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║   Energy Meter Solver - Installation Verification             ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Check if workspace is sourced
if [ -z "$ROS_PACKAGE_PATH" ]; then
    echo "⚠️  ROS environment not sourced!"
    echo "   Run: source install/setup.bash"
    exit 1
fi

echo "✓ ROS environment sourced"

# Check if package is installed
if ! ros2 pkg prefix energy_meter_solver >/dev/null 2>&1; then
    echo "✗ energy_meter_solver package not found"
    exit 1
fi
echo "✓ Package found: $(ros2 pkg prefix energy_meter_solver)"

# Check executable
EXEC_PATH="$(ros2 pkg prefix energy_meter_solver)/lib/energy_meter_solver/energy_meter_solver_node"
if [ -x "$EXEC_PATH" ]; then
    echo "✓ Executable exists: $EXEC_PATH"
else
    echo "✗ Executable not found or not executable"
    exit 1
fi

# Check config file
CONFIG_PATH="$(ros2 pkg prefix energy_meter_solver)/share/energy_meter_solver/config/params.yaml"
if [ -f "$CONFIG_PATH" ]; then
    echo "✓ Config file found: $CONFIG_PATH"
else
    echo "✗ Config file not found"
    exit 1
fi

# Check launch file
LAUNCH_PATH="$(ros2 pkg prefix energy_meter_solver)/share/energy_meter_solver/launch/energy_meter_solver.launch.py"
if [ -f "$LAUNCH_PATH" ]; then
    echo "✓ Launch file found: $LAUNCH_PATH"
else
    echo "✗ Launch file not found"
    exit 1
fi

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║   All checks passed! Ready to use.                            ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "Quick start:"
echo ""
echo "  1. In terminal 1:"
echo "     ros2 launch energy_meter_solver energy_meter_solver.launch.py"
echo ""
echo "  2. In terminal 2 (to monitor):"
echo "     ros2 topic echo energy_meter_solver/cmd_gimbal"
echo ""
echo "Documentation:"
echo "  - README.md: Full documentation"
echo "  - QUICKSTART.md: Quick start guide"
echo "  - config/params.yaml: Parameter reference"
echo ""
