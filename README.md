**This file provides the setup and installation guide for running the HMI. For instructions on how to add new features to the HMI and further development, see the separate DEVELOPER_GUIDE.md file.**

## You can run the HMI workspace either using Docker or directly on your local system. Both methods are described below. But use Docker or you're a noob.

---------------------------------------------------------
# Docker guide:

Firstly make sure you have docker and docker compose installed.

# First build the image using docker build
'''bash
docker build -t p5-hmi-ws:latest .
'''

There are two methods to running the hmi

# Method 1: Manual docker composing and such
You have to do this the first time running it and if you delete the directory kivy_venv. Afterwards you can use method 2 which is easier and quicker and nicer.

1. Remember xhost
'''bash
xhost +local:root
'''

2. Run docker compose
'''bash
docker compose up
'''

3. Exec into the container
'''bash
docker exec -it hmi-p5-p5_hmi-1 bash
'''

4. Run script to launch hmi
'''bash
./start_hmi.sh
'''

# Method 2: Use the script
This can only be done if method 1 has been done at least once. or if you run the setup script inside the container and source manually the first time.

1. Run the script
'''bash
./make_hmi_container.sh
'''

2. Run hmi launch script
'''bash
./start_hmi.sh
'''

# INSTALLATION GUIDE FOR RUNNING THE HMI ON YOUR LOCAL SYSTEM!!!

The HMI is built using the Kivy and KivyMD Python frameworks, and runs on ROS2. Before you can run the HMI, you need to install both Kivy and KivyMD, and make sure ROS2 Jazzy is installed and sourced on your system.

The following installation is documented here:
- kivy: https://kivy.org/doc/stable/gettingstarted/installation.html
- kivyMD: https://kivymd.readthedocs.io/en/1.1.1/getting-started/ 

Note: Before Kivy can be installed, Python and pip needs to be pre-installed on your system.
Note: Linux users you may have to add a --user flag in the subsequent commands outside the virtual environment


# 1. Start af new terminal and run:
```bash
python3 -m pip install --upgrade pip setuptools virtualenv
```

# 2. Create virtual envoirment
Create a new virtual environment for your Kivy project. A virtual environment will prevent possible installation conflicts with other Python versions and packages. It’s optional but STRONGLY recommended:

```bash
python3 -m venv kivy_venv
```
Note: It is very important that the virtual environment is installed in the root of the HMI workspace. Otherwise, the startup script won't be able to find your installation! 😢


# 3. Activate the virtual environment. 
You will have to do this step from the current directory every time you start a new terminal. This sets up the environment so the new kivy_venv Python is used.
```bash
source kivy_venv/bin/activate
```
Your terminal should now preface the path with something like (kivy_venv), indicating that the kivy_venv environment is active. If it doesn’t say that, the virtual environment is not active and the following won’t work.

# 4. Install Kivy
```bash
python -m pip install "kivy[base]" kivy_examples
```

# 5. Install kivyMD
Now install kivyMD:
```bash
pip install kivymd
```

# 6. After this is done you need to install additional tools (inside the venv)
```bash
pip install numpy lark empy setuptools pyyaml catkin_pkg
```
WUHUUU your installation is now done!! (hopefully😈😈) 


# 7. You can now build the ROS2 workspace:
```bash
cd <path-to-hmi-ws>
colcon build
```

Then start the HMI:
```bash
source kivy_venv/bin/activate
./start_hmi.sh
```

Note: The script assumes that `kivy_venv` is located in the repo root and that the workspace has been built (`colcon build`).

Afterwards, please read the DEVELOPER_GUIDE.md file!!!

----------------

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

If you have permission denied use:
```bash
sudo docker exec -it p5_hmi_ws-p5_hmi-1 bash
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


