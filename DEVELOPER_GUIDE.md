# Developer Guide for HMI Workspace

This guide provides instructions for developers who want to contribute to or extend the HMI workspace.

## Project Structure
- `src/` — Source code for the HMI and ROS2 packages
- `kivy_venv/` — Python virtual environment for Kivy/KivyMD
- `Dockerfile` — Container build instructions
- `start_hmi.sh` — Script to launch the HMI
- `README.md` — Setup and user instructions

- The main entry point for the HMI is `src/p5_hmi/hmi.py`.
    - All main event loop handling is managed here.

## Packages Overview

This workspace contains two main ROS2 packages:

- **p5_hmi**: The main Human-Machine Interface application. Contains all GUI logic, page modules, and Kivy/KivyMD integration.
- **p5_interfaces**: Custom ROS2 message and service definitions used for communication between the HMI and other ROS2 nodes.

When developing new features, most changes will be in the `p5_hmi` package. If you need new message or service types for ROS2 communication, add them to `p5_interfaces` and rebuild the workspace with `colcon build`.


### Adding Features
- The HMI is organized into "pages". Each page represents a distinct view or feature in the GUI.
- For each page, there is:
    - A Python file in `src/p5_hmi/pages/` (e.g., `my_page.py`) containing the logic for that page.
    - A corresponding Kivy layout file in `src/p5_hmi/kv/` (e.g., `my_page.kv`) defining the UI for that page.
- To add a new feature:
    1. Create a new Python file for your page in `src/p5_hmi/pages/`.
    2. Create a matching `.kv` file in `src/p5_hmi/kv/` for the UI.
    3. Register your new page in the main app logic (`hmi.py`).
    4. Keep all feature-specific logic and UI in these files to maintain modularity.

- When extending an existing feature, update the relevant page's `.py` and `.kv` files.


## Documentation Links

- [Kivy Documentation](https://kivy.org/doc/stable/)
- [KivyMD Documentation](https://kivymd.readthedocs.io/en/latest/)

## Useful Commands
- Run HMI: `./start_hmi.sh`
- Activate venv: `source kivy_venv/bin/activate`
- Simulate an error message send from the server: `ros2 topic pub /error_messages p5_interfaces/msg/Error "{stamp: {sec: 0, nanosec: 0}, severity: 'FATAL', message: 'Jensens kode fejler fuldstændig', node_name: 'test_node'}" --once`

## Hardware Note: Raspberry Pi Touch Interface

The HMI is set to run on a Raspberry Pi with a touch interface running Ubuntu Server 24.04. The GUI-interface installed on the Rasberry Pi is XFCE4

- Device code: 123
- The workspace code is located at `/Desktop/P5-hmi-ws` on the rasberry pi

Note: Docker is not installed on the Rasberry Pi. All dependencies are installed locally. For instructions on how to run the HMI on the Rasberry pi, see the README.md file, under locally setup


Happy coding! May your bugs be few.