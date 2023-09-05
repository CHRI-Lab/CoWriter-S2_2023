# NA-Boxjelly
Repository for work by the NA-Boxjelly team on the ChatGPT and Nao Robot project.

# Table of Contents
- [Table of Contents](#table-of-contents)
- [Stakeholders](#stakeholders)
- [Overview](#overview)
- [Workflow](#workflow)
- [Repository Structure](#repository-structure)
- [Developer Guide](#developer-guide)
  - [Environment Installation Guide](#environment-installation-guide)
  - [Virtual Machine Setup Guide](#virtual-machine-setup-guide)
  - [Troubleshooting](#troubleshooting)
  - [State Machine Diagram](#state-machine-diagram)
  - [ROS Topic Graph](#ros-topic-graph)
- [User Guide](#user-guide)
  - [Choregraph Simulator Guide](#choregraph-simulator-guide)
  - [Nao Robot Startup Guide](#nao-robot-startup-guide)
  - [Program Launch Guide](#program-launch-guide)

**Important Notice: Currently the OpenAI API key being used is the personal key of a developer who worked on this project. This key will be cancelled at the end of June 2023, so will need to be replaced by then for the ChatGPT functionality to work; the file it needs to be replaced in is nao_settings.py***

# Stakeholders
*Client:* Wafa Johal

*Supervisor:* Max Plumley

*Development Team*
- Joel Towell
- Jun Li Chen
- Chang Shen
- Yanting Mu
- Tianyang Wang

# Overview  

This project aims to modernise and enhance the CoWriter Letter Learning program by updating its software stack, incorporating Python 3.8.10, the latest version of ROS (Noetic), and integrating ChatGPT, for a more personalised and engaging learning experience. 

The primary goals of this project are as follows:

    1. Update the Python version from 2.7 to 3.8.10 for improved compatibility, performance, and security.

    2. Transition from older ROS versions to ROS Noetic for better compatibility with newer hardware and access to the latest ROS packages and tools.

    3. Integrate ChatGPT into the dialogue manager to faciliate dynamic, natural, and personalised conversations between children participating in the CoWriter program and the Nao robot. 

    4. Leveraging ChatGPT's capabilities to generate word suggestions based on children's interests, thereby creating a more customised and motivating learning environment. 

This repository will contain all the necessary resources, documentation, and code for updating the CoWriter program and integrating ChatGPT.

The existing project and dependencies can be found in the following locations (note: these are the old versions and will be updated as part of the current project):

Main project:
- cowriter: https://github.com/CHRI-Lab/cowriter
- cowriter_letter_learning: https://github.com/CHRI-Lab/cowriter_letter_learning

Dependencies:
- nao_writing: https://github.com/chili-epfl/nao_writing
- shape_learning: https://github.com/chili-epfl/shape_learning

# Workflow 

The naming conventions in all python files will follow the PEP 8 style guidelines. 

Naming conventions for other files in src will largely be enforced by ROS (CMakeLists.txt, package.xml, etc.).

The branching strategy used to develop this project will be GitHub Flow. As such, the naming convention for branches other than main will be to use short descriptive branch names which state the feature they are being used to develop.

# Repository Structure 
```
.
├── build                           # catkin_make build files
├── devel                           # catkin_make devel files
├── docs                            # documentation files
├── share                           # dataset files
├── src                             # ROS packages source code
│   ├── cowriter_letter_learning
│   ├── choose_adaptive_words
│   ├── nao_trajectory_following
│   └── packages                    # other dependencies
│       ├── ros_markers
│       └── shape_learning
└── CHANGELOG.md
├── README.md
├── requirements.txt                # python dependencies
└── setup.sh                        # setup file (see installation section below)
```

# Developer Guide

## Environment Installation Guide

These instructions assume the user is not running Ubuntu 20.04 as their main operating system. Those who are can skip the steps relating to the setup of Ubuntu on a virtual machine; however, these users should be conscious about setting up a fresh Python virtual environment for this project, as there may be dependency conflicts between this project and packages installed on their default system Python.

## Virtual Machine Setup Guide
First, download VirtualBox for your OS, we recommend using 7.0.8 as this is what the team is using. You can download it here: https://www.virtualbox.org/wiki/Downloads

Next, you need to download the ISO file for Ubuntu 20.04.6, which you can find here: https://releases.ubuntu.com/focal/

The next step is to create a Ubuntu 20.04 virtual machine on VirtualBox, please see here for instructions (steps 2 and 3 are most relevant here - note it is recommended to install with Guest Additions): https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#2-create-a-new-virtual-machine


### For Mac Users

You will need to install AVX2 support, run the following in your VM terminal:
```
sudo apt update 
sudo apt install libmkl-dev libmkl-avx2 -y
	# (option page: ok, no)
```
### Step 0
Clone the repository:
```
sudo apt install git -y
git clone https://github.com/COMP90082-2023-SM1/NA-Boxjelly.git
```
### Step 1a Auto installation

For simplicity's sake, we've created a setup.sh file that automatically setup and verify your development environment, build necessary dependencies, project packages, and optionally allows you to run test files.
```
cd <path-to-NA-Boxjelly>
./setup.sh
# follow on-screen options to select 1 (Yes) or 2 (No) when asked to run test cases
```
**IMPORTANT:  replace `<path-to-NA-Boxjelly>` with the path to your `/NA-Boxjelly` directory.**
### Step 1b Manual installation
If something is wrong with the auto installation, the following steps will show how to do the installation manually.

First setup the ROS Noetic package by running:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install git python3-pip ros-noetic-desktop-full -y
echo "export PATH="$PATH:$HOME/.local/bin"" >> ~/.profile
```
You can also checkout the more detailed [installation guide](http://wiki.ros.org/Installation/Ubuntu).
### Step 2
Once ROS Noetic is set up, you must run the following source command in every bash terminal you are going to use ROS in:
```
source /opt/ros/noetic/setup.bash 
```
If you want to have multiple terminals open when using ROS Noetic, it is useful to add this source command to your .bashrc file as follows:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
source ~/.bashrc
```
### Step 3
Install dependencies for building ROS packages:
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

### Step 4
Required Python packages installation:
```
cd <path-to-NA-Boxjelly>
pip install -r requirements.txt
```

### Step 5
Install the packages needed to build boost from source:
```
pip3 install qibuild scikit-build toml
sudo apt install libgmock-dev patchelf -y
echo "PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
source ~/.bashrc
```
### Step 6
Boost 1.77.0 is needed for us to build libqi-python from source.
Download boost from this [link](https://boostorg.jfrog.io/artifactory/main/release/1.77.0/source/boost_1_77_0.tar.bz2), and extract it by running the following:
```
curl -L https://boostorg.jfrog.io/artifactory/main/release/1.77.0/source/boost_1_77_0.tar.bz2 -o boost_1_77_0.tar.bz2
tar -xjf boost_1_77_0.tar.bz2
cd boost_1_77_0
./bootstrap.sh
./b2 --with=all -j 4 install --prefix=$HOME/boost_1_77_0
echo "export BOOST_ROOT=$HOME/boost_1_77_0" >> ~/.bashrc
echo "export Boost_INCLUDE_DIR=$HOME/boost_1_77_0/include" >> ~/.bashrc
echo "export Boost_LIBRARY_DIR=$HOME/boost_1_77_0/lib" >> ~/.bashrc
```

### Step 7
Build and install libqi-python from source by following the code below:
```
git clone https://github.com/aldebaran/libqi-python.git
cd libqi-python
sed -i "s+'-DQIPYTHON_STANDALONE:BOOL=ON'+'-DQIPYTHON_STANDALONE:BOOL=ON', '-DBOOST_ROOT:PATH=$HOME/boost_1_77_0', '-DBoost_INCLUDE_DIR:PATH=$HOME/boost_1_77_0/include', '-DBoost_LIBRARY_DIR:PATH=$HOME/boost_1_77_0/lib'+g" setup.py
python3 ./setup.py bdist_wheel -j 4
cd dist
pip3 install <qi.whl file>
```
**IMPORTANT:  replace `<username>` with your ubuntu username.**
**IMPORTANT:  replace `<qi.whl file>` with the qi.whl file found in `/dist`.**

### Step 8
Build the CoWriter package by running the following:
```
cd <path-to-NA-Boxjelly>

rm -rf build/*
rm -rf devel/*

catkin_make --only-pkg-with-deps chilitags_catkin
catkin_make --only-pkg-with-deps ros_markers
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

source <path-to-NA-Boxjelly>/devel/setup.bash

# run this once after catkin_make is run to ensure the 
# build files can be sourced when launching a new terminal
echo "source <path-to-NA-Boxjelly>/devel/setup.bash" >> ~/.bashrc

# setup the database path in src/catkin_ws/src/choose_adaptive_words/nodes/parameters.py
# PATHDB='<path-to-NA-Boxjelly>/ui_database'"
```

### (Optional) Step 9
Run test cases:
```
cd <path-to-NA-Boxjelly>
catkin_make run_tests && catkin_test_results build/test_results
```

### Step 10
Install Choregraphe, the NAO robot simulator:

Download ubuntu setup.run file in https://www.aldebaran.com/en/support/nao-6/downloads-softwares
```
cd Downloads
chmod +x choregraphe-suite-2.8.6.23-linux64-setup.run
sudo ./choregraphe-suite-2.8.6.23-linux64-setup.run
```

Run the simulator:
```
/opt/'Softbank Robotics'/'Choregraphe Suite 2.8'/bin/choregraphe_launcher
```

In case of error:
```
/opt/Softbank Robotics/Choregraphe Suite 2.8/bin/../lib/../lib/../lib/libz.so.1:
version `ZLIB_1.2.9' not found (required by /usr/lib/x86_64-linux-gnu/libpng16.so.16)
```

run the following:
```
cd /opt/'Softbank Robotics'/'Choregraphe Suite 2.8'/lib/
sudo mv libz.so.1 libz.so.1.old
sudo ln -s /lib/x86_64-linux-gnu/libz.so.1
```

## Troubleshooting

### ROS Noetic Installation Isn't Working
You may need to delete the key and add a different one before it will install, try running the following which should resolve the issue.
```
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt clean && sudo apt update 
sudo apt install ros-noetic-desktop-full 
```

### You Don't Have sudo Privileges on your Virtual Machine
Please follow these steps:
```
su
visudo
```
- Under the heading ‘# User privilege specification’ add the line: \<username\> ALL=(ALL:ALL) ALL
- Save and exit the permissions text file
```
exit
sudo -l
# <user password>
```
This should confirm that you can run sudo commands. required sudo privileges.

### No AVX2 support:
Due to the need for AVX2 support to run the VM on Mac, if the host machine has HyperV enabled, we will need to turn it off. (if using VirtualBox, should see a V on a chip (AMD-V), instead of a V on a turtle (HyperV) on button right of the window)

If you see "V on a turtle", in host machine terminal: 

```
bcdedit /set hypervisorlaunchtype off
DISM /Online /Disable-Feature:Microsoft-Hyper-V
# (restart host machine)
```

### Enabling Full-Screen for Virtual Box
```
sudo apt install build-essential gcc make perl dkms
y
```
1. Reboot the Ubuntu VM (power down then log back on)
2. In the top left of the VirtualBox window, click the 'Devices' button
3. From the drop-down menu, click ‘Insert Guest Additions CD Image..’
4. A prompt should come up on the Ubuntu screen asking if you want to run BVox_GAs_7.0.6, select 'Run'
5. Power down the VM then start it again and log in
6. Try minimise then maximise the screen, with some luck it will go full screen
7. If this has not worked, right click the CD-ROM icon on the left toolbar and click 'Eject'
8. Repeat steps 2-6, if it didn't work the first time, it should work the second

### Enabling Shared Clipboard Between Native OS and Virtual Machine 

Note: It is useful, but not strictly necessary, to share a clipboard between your VM and normal OS. To enable this, click 'Devices' in the top left of the VirtualBox widow, hover your cursor over 'Shared Clipboard' then select 'Bidirectional'.

## State Machine Diagram
<center><img src="./docs/img/state_machine.jpg?raw=true" width=500px alt="State Machine Diagram"></center>

## ROS Topic Graph
<center><img src="./docs/img/rqt_graph.png?raw=true" width=643px alt="ROS Topic Graph"></center>

# User Guide

## Choregraph Simulator Guide
To run the Choregraphe simulator, please click the icon of the software or run the following command line in the terminal.

```
/opt/'Softbank Robotics'/'Choregraphe Suite 2.8'/bin/choregraphe_launcher
```
After starting the simulator for the first time, remember to adjust the settings of the robot version：

Click **Edit** then click **Preferences**, a new window will be opened.

Go to the **Virtual Robot** page, change the **Robot model** to **NAO H25(V6)**, then click **OK** in the bottom right corner. The program will ask to restart the robot, click **Yes**.

The simulator will automatically generate a virtual robot,  click the connection button ![Connect to... button](./docs/img/connect_button.png?raw=true) to check the connection list.

After waiting for a while, a virtual robot will appear in the connection list.
<center><img src="./docs/img/connection_list.png?raw=true" width=400px alt="Connection list popup window"></center>

Select the virtual robot, after the robot is connected, make sure to click the play button ![Upload to the robot and play button](./docs/img/play_button.png?raw=true).

The default IP address and port of the virtual robot is 127.0.0.1 and 9559.

Now the robot is waiting for the connection from the program.

## Nao Robot Startup Guide
The Nao robot can connect to the network through a wired connection to the interface behind their head, or wirelessly to wifi.

It is recommended to use a wired connection and check the robot's settings when starting the robot for the first time.

To start the robot, press the button on its chest. After the robot starts, pressing that button again will tell you its IP address through voice.

When both the computer and robot are connected to the same network, entering the robot's IP address in the browser will enter the robot's settings page, with the default login username and password being `nao`.

Here, the robot's network connection can be adjusted to connect to a network with internet access.

<center>
  <p align="center">
    <img src="./docs/img/nao_front.png?raw=true" width=281px alt="Nao Robot Front"> <img src="./docs/img/nao_back.png?raw=true" width=500px alt="Nao Robot Back">
    <br>
    Startup button and interface
  </p>
</center>


*It should be noted that every startup or switch of network connection may cause a change in the robot's IP address.*

*Please remember to press the button again after performing the above operations to confirm whether the IP address has changed.*

## Program Launch Guide

### Step 1

Make sure `roscore` is running, open a terminal and run the following command line:
```
roscore
```

### Step 2
Open the manager UI:
```
cd <path-to-NA-Boxjelly>
python3 src/choose_adaptive_words/nodes/manager_ui.py
```
The manager UI window will open.
<center><img src="./docs/img/manager_UI.png?raw=true" width=700px alt="Manager UI Window"></center>

**IMPORTANT:  replace `<path-to-NA-Boxjelly>` with the path to your `/NA-Boxjelly` directory.**

### Step 3
Open a second terminal to run the child UI:

```
cd <path-to-NA-Boxjelly>
python3 src/choose_adaptive_words/nodes/child_ui.py
```
The child UI window will open.
<center><img src="./docs/img/child_UI.png?raw=true" width=700px alt="Child UI Window"></center>

### Step 4
Open a third terminal and run temporary UI backend:
```
cd <path-to-NA-Boxjelly>
python3 src/choose_adaptive_words/nodes/temp_backend.py
# manager UI and child UI are connected to the temporary UI backend
```

### Step 5
Open a fourth terminal and run the CoWriter program:
```
cd <path-to-NA-Boxjelly>
roslaunch letter_learning_interaction nao_learning.launch letter_model_dataset_directory:=<path-to-NA-Boxjelly>/share/letter_model_datasets/<model datasets>  #such as alexis_set_for_children
```

When the program is successfully connected to the robot, there will be voice from the real robot or a dialog box from the simulated robot, saying `Hi, I'm nao, please give me a word to practice.`.
Now the program and robot is ready for the operations.

### Operation 1: Writing and Learning from the user

1.  Click the **Send new word** button in the manager UI, the robot will generate writing trajectory for the specified word or letter, then write it with it's right hand. The trajectory will also be shown in the child UI.

2. Click the **Robot finished** button, then the robot will be ready for learning from the user's writing, the user should write in the child UI and click the paper plane logo in the upper right corner to send the new trajectory, or click the eraser logo to clear the window and rewrite. The robot will learn from the human writing and will change the trajectory of the same word or letter next time.

[![](https://res.cloudinary.com/marcomontalbano/image/upload/v1685882545/video_to_markdown/images/youtube--C47BLQqLA1M-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/C47BLQqLA1M "")

<a href=https://youtu.be/C47BLQqLA1M>https://youtu.be/C47BLQqLA1M</a>

### Operation 2: Communicating with the user

There are two ways to communicate with the robot: 
1. By typing a sentence into the `Chat with me` box.
2. By clicking the `Talk To Me` button and then speaking into the microphone of the computer. 

The robot will respond after processing voice or text input.

[![](https://res.cloudinary.com/marcomontalbano/image/upload/v1685882529/video_to_markdown/images/youtube--UgzBqFEUJh8-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/UgzBqFEUJh8 "")

<a href=https://youtu.be/UgzBqFEUJh8>https://youtu.be/UgzBqFEUJh8</a>
<br>

*It should be noted that the computer needs to have internet access to use ChatGPT to generate answers, and ensure that the virtual machine can receive input from the microphone when speaking.*

These VirtualBox audio setting worked well for our team:
<center><img src="./docs/img/audio_settings.png?raw=true" width=350px alt="Virtualbox audio settings"></center>

### Usage demo
[![](https://res.cloudinary.com/marcomontalbano/image/upload/v1685882413/video_to_markdown/images/youtube--qhCjyhDt22A-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=qhCjyhDt22A "")

<a href=https://youtu.be/qhCjyhDt22A>https://youtu.be/qhCjyhDt22A</a>
