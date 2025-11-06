#!/bin/bash
# A setup script for Kivy virtual environment

# Stop on first error
#set -e

VENV_DIR="kivy_venv"

echo "Checking for existing virtual environment..."

if [ -d "$VENV_DIR" ]; then
    echo "Virtual environment '$VENV_DIR' already exists."
else
    echo "Creating new virtual environment '$VENV_DIR'..."
    python3 -m pip install --upgrade pip setuptools virtualenv --break-system-packages
    python3 -m venv "$VENV_DIR"
    echo "Virtual environment created."
    echo "Activating virtual environment..."
    # shellcheck source=/dev/null
    source "$VENV_DIR/bin/activate"

    echo "Upgrading pip inside the virtual environment..."
    pip install --upgrade pip setuptools

    echo "Installing Kivy and dependencies..."
    pip install "kivy[base]" kivy_examples

    echo "Installing KivyMD..."
    pip install kivymd

    echo "Installing additional packages..."
    pip install numpy lark empy setuptools pyyaml catkin_pkg

    echo "All installations complete!"
    echo "To activate your environment later, run:"
    echo "   source $VENV_DIR/bin/activate"

fi