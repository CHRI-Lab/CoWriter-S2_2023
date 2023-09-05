## Project Launch Steps

### configuration environment -ROS2 and Python

#### Setup Python3.10

  **Windonws** 
  1. Visit the official Python website's [download page](https://www.python.org/downloads/) Download the Windows installer for Python 3.10 by clicking on the appropriate link (either 32-bit or 64-bit, depending on your system). 
  2. Run the installer and follow the on-screen instructions.
  3. Make sure to check the box that says `Add Python 3.10 to PATH` during installation. This will make it easier to run Python from the command line.
  4. Complete the installation process.

  **macOS**
  1. Visit the official Python website's [download page](https://www.python.org/downloads/) Download the macOS installer for Python 3.10.
  2. Open the downloaded package and follow the on-screen instructions to complete the installation process.
   
  **Linux (Ubuntu, Debian)** 

  Python 3.10 may not be available in the default repositories for some Linux distributions. In that case, you can use the "deadsnakes" PPA repository to install it. Run the following commands in the terminal:
  ```
  sudo apt update

  sudo apt install software-properties-common

  sudo add-apt-repository ppa:deadsnakes/ppa

  sudo apt update

  sudo apt install python3.10
  ```
  After completing the installation, you can check the Python version by running `python3.10 --version` in the command line or terminal.

#### Setup Python 2.7

  **Windonws** 
  1. Visit the official Python website's [download page](https://www.python.org/download/releases/2.7/) Download the Windows installer for Python 2.7 by clicking on the appropriate link (either Windows x86 or Windows x86-64, depending on your system). 
  2. Run the installer and follow the on-screen instructions.
  3. Make sure to check the box that says `Add Python 2.7 to PATH` during installation. This will make it easier to run Python from the command line.
  4. Complete the installation process.

  **macOS**
  1. Visit the official Python website's [download page](https://www.python.org/download/releases/2.7/) Download the macOS installer for Python 2.7.
  2. Open the downloaded package and follow the on-screen instructions to complete the installation process.
   
  **Linux (Ubuntu, Debian)** 

  Python 2.7 may not be available in the default repositories for some Linux distributions. In that case, you can use the "deadsnakes" PPA repository to install it. Run the following commands in the terminal:
  ```
  sudo apt update
  sudo apt install python2
  ```
  After completing the installation, you can check the Python version by running `python --version` in the command line or terminal.


#### Setup ROS2 Humble

**Ubuntu Linux**

```
sudo apt update sudo apt upgrade

sudo locale-gen en_US en_US.UTF-8 sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 export LANG=en_US.UTF-8

sudo apt install curl gnupg2 lsb-release curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

sudo apt update

sudo apt install ros-humble-desktop

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc source ~/.bashrc
```
**macOS and Windows**

ROS2 is primarily targeted at Ubuntu Linux, but there are experimental installations available for macOS and Windows. The official ROS2 installation documentation provides instructions for these platforms, but keep in mind that support may be limited, and not all features and packages may be available. For the most up-to-date instructions, refer to the official ROS2 documentation: [macOS](https://index.ros.org/doc/ros2/Installation/OSX-Development-Setup/) and [Windows](https://index.ros.org/doc/ros2/Installation/Windows-Development-Setup/)

Remember to always check the official ROS2 documentation for the most accurate and up-to-date installation instructions.

#### Setup Docker

**Ubtuntu**

```
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

**macOS**

```
sudo yum install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo systemctl start docker
```

**Windows**
Please follow the [installation manual](https://docs.docker.com/desktop/install/windows-install/) to install the desktop version of Docker

#### Setup NA-RedBack Repository

1. Set Git username information
```
git config --global user.name  "github’s Name"
git config --global user.email  "github@xx.com"
```
2. Configure ssh key

``` 
ssh-keygen -t rsa -C "github@xx.com"
cd .ssh
```
Enter `Github`, in `Your Profile`, click on `SSH Keys` on the left, and then click on the `Add SSH key` button on the right

Among them, fill in the `title` casually,copy the content from `id_rsa.pub` to `Key`, click the `Add Key` button below, and then enter the following command on the command line to check if the configuration is successful:
```
ssh -T git@github
```
If the welcome statement is returned, the configuration is successful. If it fails, please check the previous operation process for any errors

3. Clone Repository
```
git clone git@github.com:COMP90082-2023-SM1/NA-RedBack.git

```

4. Use Docker to build project
   
```
cd NA-RedBack/src/nao_drawing_board/
docker compose -f docker-compose.dev.yml up --build
```
**Note that each time a new repository is installed, a new build is required, or you may install Node.js and run the dev server locally**

5. You may now visit the web applications with the address of the dev server

### Launch Naoqi Controller
Since the naoqi SDK is written in python 2.7, you may only import it with python 2.7.
1. Prepare python 2.7 venv
2. Get the ip address and port number of the Nao Robot and edit the code
3. Run the file `tests/Naoqi2robot/nao_bot_controller.py`
Now the robot should be connected and ready to accept input from the drawing board and the chatbox.

### Launch ROS2 Nodes
1. Navigate to the folder `src/nao_ros2_ws`
2. Build the project with command `colcon build`
3. Source the project with command `source install/setup.bash`
4. Run the launch file with `ros2 launch launch/nao_writing_launch.py`
Now, four nodes will be launched, which are `input_interpreter, learning_word, nao_writer_naoqi, audio_chat`
You should also notice a chat box also pops up on the screen.

#### Launch Naoqi Controller
Since the naoqi SDK is written in python 2.7, you may only import it with python 2.7.
1. Prepare python 2.7 venv
2. Get the ip address and port number of the Nao Robot and edit the code
3. Run the file `tests/Naoqi2robot/nao_bot_controller.py`
Now the robot should be connected and ready to accept input from the drawing board and the chatbox.

### OpenAI key setup

Since this project incorporates chatGPT, an openAI key needs to be set up to invoke the GPT model. Here are the steps to set up openAI key:

1. Create an OpenAI account. Visit the OpenAI [website](https://www.openai.com/) and follow the instructions to create a new account.

2. Log into the [website](https://platform.openai.com/account/billing/overview) to set up billing information for using OpenAI API.
 
3. Log into the [API keys website](https://platform.openai.com/account/api-keys) to create new secret key.

4. When the key is created, copy and paste it to save it. The complete key may only appear once.

5. In the project folder, nevigate to: `src/nao_ros2_ws/src/nao_writing/nao_writing/audio_chat.py`, replace the value of the global variable: `api_key` in the `audio_chat.py` file with the key you just obtained in step3


### Choregraphe installation guidence

Choregraphe is a software for programming and monitoring SoftBank Robotics' Nao and Pepper robots. Here are the steps on how to install and launch Choregraphe on Linux and Windows systems.

**Linux:**

1. Go to SoftBank Robotics' official website and download the Choregraphe suite for Linux systems.

2. Open your Linux terminal.

3. Navigate to the folder containing the `.tar.gz` file you downloaded. You can do this using the `cd` command.

4. Extract the downloaded `.tar.gz` file using the `tar -xvzf [filename].tar.gz` command.

5. Navigate into the extracted directory using the `cd` command.

6. To start Choregraphe, you can run the choregraphe script in the bin folder. The command would be `./bin/choregraphe`.

**Windows:**

1. Go to SoftBank Robotics' official website and download the Choregraphe suite for Windows systems.

2. After downloading the `.exe` file, navigate to the download location.

3. Double-click on the `.exe` file to launch the installer.

4. Follow the instructions provided by the installer.

5. Once installed, you should be able to launch Choregraphe either from the Start menu or the desktop shortcut if you chose to create one during installation.


**To switch robot model:**

Navigate to the "View" tab, find and select the "Robot View" option, then navigate to the top-right corner. You should see a drop-down menu where you can select the robot model you want to use.

### Use the drawing board
The drawing board consisted of two button and one input box, the two buttons are 'Done', 'Clear'.
'Done' button will send the drawing strokes to backend and 'Clear' button will clear all inputs.
1. Input the word you want to write in the input box, you should see eqaul numbers of canvas being created for each char
2. Draw each char as suggested by the char on top of each canvas, you may use the 'Rewrite' button to clear the corresponding canvas
3. Click 'Done' to send the drawings to backend
   
<img src='https://github.com/COMP90082-2023-SM1/NA-RedBack/blob/c3bde9e5675819aabcedbdadc0058a2ab2a1a944/prototypes/images/243603683-e9eac786-cfc6-4648-b925-d96d113b3d3e.png'/>

### Audio input, conversation with robot and collection of words guidance

The process for users to input with their own microphone is as follows:

1. The user first clicks on "record". There might be some warnings in the terminal after clicking, but it won't affect the process (this might be due to issues between the Ubuntu virtual environment and the Windows audio device).

2. After seeing the warning, the user starts to speak. When finished, the user waits for a second or two.

3. The "stop" button will light up, indicating that the parsing of user input is complete. The user clicks "stop", and the parsed user input will appear in the prompt box. Sometimes the parsing might not be accurate, in which case the user can manually modify the input.

4. The user clicks "submit" to submit the question and wait for the reply.

The implementation logic of "collection of words" is almost the same as a regular chat. The user mentions a topic of interest, and then ChatGPT returns 5 related words for the user to choose and practice. However, the difference is that, whether the user types in the input or speaks it, the phrase "let's practice" or "let's practise" must appear in the user's input. Only then will the program enter the "collection of words" mode. An example of collection of words user input request: Hi, I'm interested in space. **Let's practice** some english words about space!
