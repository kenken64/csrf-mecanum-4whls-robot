#!/bin/bash

# ELRS Robot Motor Plotter Launcher Script
# This script ensures Python environment is ready and runs the motor plotter

set -e  # Exit on any error

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLOTTER_SCRIPT="$SCRIPT_DIR/motor_plotter.py"
REQUIREMENTS_FILE="$SCRIPT_DIR/requirements.txt"
PYTHON_CMD="python3"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check Python version
check_python() {
    log_info "Checking Python installation..."

    if ! command_exists "$PYTHON_CMD"; then
        log_error "Python3 not found. Please install Python 3.6 or higher."
        exit 1
    fi

    # Check Python version
    PYTHON_VERSION=$($PYTHON_CMD --version 2>&1 | grep -oP '\d+\.\d+' | head -1)
    if [ -z "$PYTHON_VERSION" ]; then
        log_error "Could not determine Python version"
        exit 1
    fi

    # Extract major and minor version numbers
    PYTHON_MAJOR=$(echo "$PYTHON_VERSION" | cut -d. -f1)
    PYTHON_MINOR=$(echo "$PYTHON_VERSION" | cut -d. -f2)

    if [ "$PYTHON_MAJOR" -lt 3 ] || ([ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 6 ]); then
        log_error "Python $PYTHON_VERSION found, but Python 3.6+ is required."
        exit 1
    fi

    log_success "Python $PYTHON_VERSION found"
}

# Function to install Python packages
install_packages() {
    log_info "Checking Python packages..."

    # Check if pip is available
    if ! command_exists pip3 && ! $PYTHON_CMD -m pip --version >/dev/null 2>&1; then
        log_error "pip not found. Please install pip."
        exit 1
    fi

    # Use pip3 if available, otherwise use python -m pip
    if command_exists pip3; then
        PIP_CMD="pip3"
    else
        PIP_CMD="$PYTHON_CMD -m pip"
    fi

    # Check if requirements.txt exists
    if [ ! -f "$REQUIREMENTS_FILE" ]; then
        log_error "requirements.txt not found at $REQUIREMENTS_FILE"
        exit 1
    fi

    # Check if packages are already installed
    PACKAGES_INSTALLED=true
    while IFS= read -r package; do
        # Skip empty lines and comments
        [[ -z "$package" || "$package" =~ ^[[:space:]]*# ]] && continue

        # Extract package name (before any version specifier)
        package_name=$(echo "$package" | sed 's/[>=<].*//')

        if ! $PYTHON_CMD -c "import $package_name" 2>/dev/null; then
            PACKAGES_INSTALLED=false
            break
        fi
    done < "$REQUIREMENTS_FILE"

    if [ "$PACKAGES_INSTALLED" = true ]; then
        log_success "All required packages are already installed"
        return
    fi

    log_info "Installing required packages..."

    # Try different installation methods
    if $PIP_CMD install --user -r "$REQUIREMENTS_FILE" 2>/dev/null; then
        log_success "Packages installed successfully with --user flag"
    elif $PIP_CMD install --break-system-packages -r "$REQUIREMENTS_FILE" 2>/dev/null; then
        log_warning "Packages installed with --break-system-packages (use with caution)"
    else
        log_error "Failed to install packages automatically."
        echo ""
        echo "Please install the required packages manually:"
        echo "  pip3 install --user -r requirements.txt"
        echo "  # or"
        echo "  pip3 install --break-system-packages -r requirements.txt"
        echo "  # or create a virtual environment:"
        echo "  python3 -m venv venv && source venv/bin/activate && pip install -r requirements.txt"
        exit 1
    fi
}

# Function to check if plotter script exists
check_plotter_script() {
    log_info "Checking plotter script..."

    if [ ! -f "$PLOTTER_SCRIPT" ]; then
        log_error "Motor plotter script not found at $PLOTTER_SCRIPT"
        exit 1
    fi

    if [ ! -x "$PLOTTER_SCRIPT" ]; then
        log_warning "Plotter script is not executable, making it executable..."
        chmod +x "$PLOTTER_SCRIPT"
    fi

    log_success "Plotter script found and ready"
}

# Function to run the plotter
run_plotter() {
    log_info "Starting ELRS Robot Motor Plotter..."
    echo ""
    echo "========================================"
    echo "  ELRS Robot Control - Motor Plotter"
    echo "========================================"
    echo ""
    echo "Controls:"
    echo "  - Press Ctrl+C to stop"
    echo "  - Use --demo flag for simulated data"
    echo "  - Use --port to specify serial port"
    echo ""
    echo "========================================"
    echo ""

    # Run the plotter with any passed arguments
    exec $PYTHON_CMD "$PLOTTER_SCRIPT" "$@"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "ELRS Robot Motor Plotter Launcher"
    echo ""
    echo "Options:"
    echo "  --demo          Run in demo mode with simulated data"
    echo "  --port PORT     Specify serial port (default: auto-detect)"
    echo "  --help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Run with live Arduino data"
    echo "  $0 --demo            # Run with simulated data"
    echo "  $0 --port /dev/ttyUSB0 # Use specific serial port"
}

# Main script
main() {
    echo ""
    echo "========================================"
    echo "  ELRS Robot Motor Plotter Launcher"
    echo "========================================"
    echo ""

    # Parse command line arguments
    DEMO_MODE=false
    SERIAL_PORT=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --demo)
                DEMO_MODE=true
                shift
                ;;
            --port)
                SERIAL_PORT="$2"
                shift 2
                ;;
            --help)
                show_usage
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                show_usage
                exit 1
                ;;
        esac
    done

    # Run setup checks
    check_python
    install_packages
    check_plotter_script

    # Prepare arguments for plotter
    PLOTTER_ARGS=()
    if [ "$DEMO_MODE" = true ]; then
        PLOTTER_ARGS+=(--demo)
    fi
    if [ -n "$SERIAL_PORT" ]; then
        PLOTTER_ARGS+=(--port "$SERIAL_PORT")
    fi

    echo ""
    log_success "Setup complete! Launching plotter..."
    echo ""

    # Run the plotter
    run_plotter "${PLOTTER_ARGS[@]}"
}

# Handle script interruption
trap 'echo ""; log_info "Plotter launcher stopped by user"; exit 0' INT TERM

# Run main function
main "$@"