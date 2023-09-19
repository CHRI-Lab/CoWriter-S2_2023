# Dockerfile of image chienpul/nao-base-lite:v4
FROM ros:noetic

# Create the nao user with sudo privileges
RUN useradd -m nao && echo "nao:nao" | chpasswd && adduser nao sudo
RUN usermod -aG audio nao

# On building machine
ENV RESOURCES_DIR=./resources

# Declaration of environment variables
ENV MAIN_DIR=/home/nao 
ENV BOOST_EXTRACTION_DIR=/tmp/boost_1_77_0 
ENV BOOST_ROOT_DIR=${MAIN_DIR}/boost_1_77_0 
ENV BOOST_INCLUDE_DIR=${BOOST_ROOT_DIR}/include 
ENV BOOST_LIB_DIR=${BOOST_ROOT_DIR}/lib 
# ENV LIBQI_BUILD_DIR=/tmp/libqi 
# ENV LIBQI_ROOT_DIR=${MAIN_DIR}/libqi 
ENV CATKIN_DIR=${MAIN_DIR}/catkin_ws 
ENV CATKIN_SRC_DIR=${CATKIN_DIR}/src 
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages

# Install system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-interactive-markers \
    ros-${ROS_DISTRO}-audio-common \
    python3-pip git cmake \
    libyaml-cpp-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    alsa-base alsa-utils libsndfile1-dev \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 \
    libxcb-shape0 libxcb-xinerama0 libxcb-xkb1 libxkbcommon-x11-0
    # ros-${ROS_DISTRO}-ros-tutorials \
    # ros-${ROS_DISTRO}-common-tutorials \
    # libgmock-dev patchelf

RUN echo "source /opt/ros/noetic/setup.bash" >> ${MAIN_DIR}/.bashrc

# Install Python packages
RUN pip3 install qibuild scikit-build toml \
    matplotlib openai google-cloud scikit-learn recordtype \
    --upgrade google-api-python-client \
    --upgrade google-cloud-speech

# Boost installation
COPY ${RESOURCES_DIR}/boost_1_77_0 ${BOOST_EXTRACTION_DIR}
WORKDIR ${BOOST_EXTRACTION_DIR}
RUN ./bootstrap.sh && \
    ./b2 --with=all -j2 install --prefix=${BOOST_ROOT_DIR}

# libqi install
# WORKDIR /tmp
# RUN git clone https://github.com/ros-naoqi/libqi
# WORKDIR ${LIBQI_BUILD_DIR}/BUILD
# ENV CMAKE_PREFIX_PATH=/opt/ros/noetic
# RUN cmake .. -DQI_WITH_TESTS=OFF -DBOOST_ROOT=${BOOST_ROOT_DIR} -DBoost_INCLUDE_DIR=${BOOST_INCLUDE_DIR} -DBoost_LIBRARY_DIR=${BOOST_LIB_DIR} && \
#     make install DESTDIR=${LIBQI_ROOT_DIR}

# LIBQI PYTHON INSTALL
WORKDIR /tmp
RUN git clone https://github.com/aldebaran/libqi-python
WORKDIR /tmp/libqi-python
RUN sed -i '66c\cmake_args=["-DQIPYTHON_STANDALONE:BOOL=ON", "-DBOOST_ROOT:PATH=/home/nao/boost_1_77_0", "-DBoost_INCLUDE_DIR:PATH=/home/nao/boost_1_77_0/include", "-DBoost_LIBRARY_DIR:PATH=/home/nao/boost_1_77_0/lib"],' \
    setup.py
RUN python3 ./setup.py bdist_wheel
RUN pip3 install ./dist/qi-3.1.1-cp38-cp38-linux_x86_64.whl

# CLEAN
RUN apt-get clean && rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/* && rm -rf /root/.cache/*
RUN rm -rf ${MAIN_DIR}/boost_1_77_0