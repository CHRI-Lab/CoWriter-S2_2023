#!/bin/bash

CYAN='\033[0;36m'
RED='\033[0;31m'
BOLD=$(tput bold)
NORMAL=$(tput sgr0)
NC='\033[0m' # No Color
CURRENT_DIR=$(pwd)
ROS_INSTALLATION_LINK="http://wiki.ros.org/Installation/Ubuntu"
BOOST_DOWNLOAD_LINK="https://boostorg.jfrog.io/artifactory/main/release/1.77.0/source/boost_1_77_0.tar.bz2"
BOOST_FILENAME="boost_1_77_0"
BOOST_FILE="boost_1_77_0.tar.bz2"
AVX_SUPPORT_FLAG=$(lscpu | grep avx2)
AVX_WORKAROUND_FLAG=$(dpkg -l | grep -E "libmkl-avx2|libmkl-dev")
ROS_INSTALLED_FLAG=$(dpkg -l | grep "ros-noetic-rospy")
PYTHON_FLAG=$(dpkg -l | grep python3-pip)
LIBQI_FLAG=$(pip3 list | grep "qi ")
SOURCE_SETUP_FLAG=$(cat ~/.bashrc | grep "source "$CURRENT_DIR"/devel/setup.bash")
NUM_THREADS=$(nproc)

sudo true

echo -e "${CYAN}${BOLD}Checking AVX support...${NC}${NORMAL}"

if [ -z "$AVX_SUPPORT_FLAG" ] && [ -z "$AVX_WORKAROUND_FLAG" ]; then
    echo -e "${RED}${BOLD}Your machine doesn't support AVX instructions."
    echo -e "${RED}${BOLD}Please read the troubleshooting section in the repository's README on how to fix this."
    sleep 5
else
    echo -e "${NC}${NORMAL}AVX supported/workaround installed."
fi

echo -e "${CYAN}${BOLD}Checking ROS Noetic installation...${NC}${NORMAL}"

if [ -z "$ROS_INSTALLED_FLAG" ]; then
    echo -e "${RED}${BOLD}ROS Noetic is not installed..."
    echo -e "${NC}${NORMAL}Installing ROS Noetic..."
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full -y
    source /opt/ros/noetic/setup.bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
    sudo rosdep init
    rosdep update
    source ~/.bashrc
else
    echo -e "${NC}${NORMAL}ROS Noetic installed."
fi

echo -e "${CYAN}${BOLD}Checking Python installation...${NC}${NORMAL}"

if [ -z "$PYTHON_FLAG" ]; then
    sudo apt install python3-pip -y
    echo "export PATH="\$PATH:$HOME/.local/bin"" > ~/.profile
else
    echo -e "${NC}${NORMAL}Python installed."
fi

echo -e "${CYAN}${BOLD}Checking libqi installation...${NC}${NORMAL}"

if [ -z "$LIBQI_FLAG" ]; then
    pip3 install qibuild scikit-build toml
    sudo apt install libgmock-dev patchelf -y
    echo "PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
    source ~/.bashrc

    echo -e "${CYAN}${BOLD}Installing boost 1.77.0...${NC}${NORMAL}"
    cd
    BOOST_DIR_FLAG=$(ls | grep boost)
    if [ -z "$BOOST_DIR_FLAG" ]; then
        curl -L $BOOST_DOWNLOAD_LINK -o $BOOST_FILE
        tar -xjf $BOOST_FILE
        rm $BOOST_FILE
        cd $BOOST_FILENAME
        ./bootstrap.sh
        ./b2 --with=all -j $NUM_THREADS install --prefix=$HOME/boost_1_77_0
        echo "export BOOST_ROOT=$HOME/boost_1_77_0" >> ~/.bashrc
        echo "export Boost_INCLUDE_DIR=$HOME/boost_1_77_0/include" >> ~/.bashrc
        echo "export Boost_LIBRARY_DIR=$HOME/boost_1_77_0/lib" >> ~/.bashrc
    fi

    echo -e "${CYAN}${BOLD}Installing libqi-python...${NC}${NORMAL}"
    cd
    LIBQI_PYTHON_DIR_FLAG=$(ls | grep libqi-python)
    if [ -z "$LIBQI_PYTHON_DIR_FLAG" ]; then
        git clone https://github.com/aldebaran/libqi-python.git
        cd libqi-python
        sed -i "s+'-DQIPYTHON_STANDALONE:BOOL=ON'+'-DQIPYTHON_STANDALONE:BOOL=ON', '-DBOOST_ROOT:PATH=$HOME/boost_1_77_0', '-DBoost_INCLUDE_DIR:PATH=$HOME/boost_1_77_0/include', '-DBoost_LIBRARY_DIR:PATH=$HOME/boost_1_77_0/lib'+g" setup.py
    else
        cd libqi-python
    fi
    python3 ./setup.py bdist_wheel -j $NUM_THREADS
    cd dist
    QI_FILENAME=$(grep -rl qi)
    pip3 install $QI_FILENAME
    cd $CURRENT_DIR
fi
echo -e "${NC}${NORMAL}libqi installed."

echo -e "${CYAN}${BOLD}Installing Python modules...${NC}${NORMAL}"

pip3 install -r requirements.txt | grep -v 'already satisfied'

echo -e "${NC}${NORMAL}Python modules installed."

echo -e "${CYAN}${BOLD}Building CoWriter packages...${NC}${NORMAL}"

rm -rf build/*

rm -rf devel/*

catkin_make --only-pkg-with-deps chilitags_catkin

catkin_make --only-pkg-with-deps ros_markers

catkin_make -DCATKIN_WHITELIST_PACKAGES=""

echo -e "${NC}${NORMAL}CoWriter packages built."

echo -e "${CYAN}${BOLD}Checking setup file source...${NC}${NORMAL}"

if [ -z "$SOURCE_SETUP_FLAG" ]; then
    echo "source $CURRENT_DIR/devel/setup.bash" >> ~/.bashrc
else
    echo -e "${NC}${NORMAL}Setup file sourced."
fi

source $CURRENT_DIR/devel/setup.bash

echo -e "${CYAN}${BOLD}IMPORTANT: Please setup the database path <PATH_DB> in /src/choose_adaptive_words/nodes/parameters.py with the following path."
echo -e "${NC}${NORMAL}PATHDB='$CURRENT_DIR/ui_database'"
echo -e "${CYAN}${BOLD}Built complete."

echo -e "${NC}${NORMAL}Do you want to run test files?"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) catkin_make run_tests; 
            echo -e "${CYAN}${BOLD}Testing complete.${NC}${NORMAL}";
            catkin_test_results build/test_results;
            break ;;
        No ) echo -e "${CYAN}${BOLD} Testing cancelled.${NC}${NORMAL}";
            break ;;
    esac
done

exit 0