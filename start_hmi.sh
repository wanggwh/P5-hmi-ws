#!/bin/bash

# HMI Startup Script for ROS2
# This script sets up the environment and starts the HMI application

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory and workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

echo -e "${BLUE}=== HMI Startup Script ===${NC}"
echo -e "${YELLOW}Workspace: $WORKSPACE_DIR${NC}"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
echo -e "${BLUE}Checking prerequisites...${NC}"

if ! command_exists "python3"; then
    echo -e "${RED}Error: Python3 not found${NC}"
    echo -e "${YELLOW}Please install Python3: sudo apt install python3${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Python3 found${NC}"

if [ ! -d "/opt/ros/jazzy" ]; then
    echo -e "${RED}Error: ROS2 Jazzy not found in /opt/ros/jazzy${NC}"
    exit 1
fi

if [ ! -d "$WORKSPACE_DIR/kivy_venv" ]; then
    echo -e "${RED}Error: Virtual environment not found at $WORKSPACE_DIR/kivy_venv${NC}"
    echo -e "${YELLOW}Please create virtual environment first${NC}"
    exit 1
fi


if [ ! -f "$SCRIPT_DIR/src/p5_hmi/hmi.py" ]; then
    echo -e "${RED}Error: HMI script not found at $SCRIPT_DIR/src/p5_hmi/hmi.py${NC}"
    exit 1
fi

echo -e "${GREEN}✓ All prerequisites met${NC}"

# Setup ROS2 environment
echo -e "${BLUE}Setting up ROS2 environment...${NC}"
source /opt/ros/jazzy/setup.bash
echo -e "${GREEN}✓ ROS2 environment loaded${NC}"

# Check if workspace is built
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo -e "${BLUE}Loading workspace environment...${NC}"
    source "$WORKSPACE_DIR/install/setup.bash"
    echo -e "${GREEN}✓ Workspace environment loaded${NC}"
else
    echo -e "${YELLOW}Warning: Workspace not built. Consider running 'colcon build' first${NC}"
fi

# Activate virtual environment
echo -e "${BLUE}Activating virtual environment...${NC}"
source "$WORKSPACE_DIR/kivy_venv/bin/activate"
echo -e "${GREEN}✓ Virtual environment activated${NC}"

# Check if running in a graphical environment
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
    echo -e "${RED}Error: No graphical display detected${NC}"
    echo -e "${YELLOW}This application requires a graphical environment${NC}"
    exit 1
fi

export ROS_DOMAIN_ID=69

# Start HMI application
echo -e "${BLUE}Starting HMI application...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the application${NC}"
echo -e "${GREEN}=== HMI Application Started ===${NC}"

# Change to workspace directory
cd "$WORKSPACE_DIR"

# Start the HMI application
python3 "$SCRIPT_DIR/src/p5_hmi/hmi.py"

echo -e "${YELLOW}HMI application stopped${NC}"