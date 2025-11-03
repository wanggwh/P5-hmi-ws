
**This file provides the setup and installation guide for running the HMI. For instructions on how to add new features to the HMI and further development, see the separate DEVELOPER_GUIDE.md file.**

## You can run the HMI workspace either using Docker or directly on your local system. Both methods are described below.


# Running the HMI with Docker

## 1. Build the Docker image
```bash
cd <path-to-hmi-ws>
docker build -t p5-hmi-ws:latest .
```

## 2. Start the Docker container using docker compose
```bash
docker compose up
```

## 3. Open a new terminal and access the running Docker container
```bash
docker exec -it p5_hmi_ws-p5_hmi-1 bash
```
When you enter the container, the ROS2 Jazzy environment and the HMI workspace are automatically sourced.

The environment variable `ROS_DOMAIN_ID` should also be set to `69` by default. You can check this inside the container with:
```bash
echo $ROS_DOMAIN_ID
```
If the output is not `69`, set it manually:
```bash
export ROS_DOMAIN_ID=69
```

## 4. Activate the virtual environment (Kivy/KivyMD)
```bash
source kivy_venv/bin/activate
```
After running this command, your terminal prompt should show (kivy_venv), indicating that the virtual environment is active.


## 5. Build the ROS2 workspace (inside the container)
```bash
colcon build
```

## 6. Start the HMI application (inside the container)
```bash
./start_hmi.sh
```

## Additional:
If you have trouble displaying the GUI from the Docker container, you may need to run the following command in a new terminal (outside the container, but after it is runnning) 
```bash
xhost +local:root
```
This allows the Docker container (running as root) to access your server for GUI applications.


------------------

# Running the HMI without Docker

If you want to run the HMI outside Docker, you must first install Kivy and KivyMD on your local system.

Follow the official installation guide for Kivy here:
https://kivy.org/doc/stable/gettingstarted/installation.html

Once Kivy is installed, follow the KivyMD installation guide here:
https://kivymd.readthedocs.io/en/latest/getting-started/

After both libraries are installed, you can build the ROS2 workspace:
```bash
cd <path-to-hmi-ws>
colcon build
```

Then start the HMI GUI:
```bash
source kivy_venv/bin/activate
./start_hmi.sh
```

Note: The script assumes that `kivy_venv` is located in the repo root and that the workspace has been built (`colcon build`).

