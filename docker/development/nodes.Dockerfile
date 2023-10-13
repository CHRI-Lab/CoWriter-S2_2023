# syntax=docker/dockerfile:1

FROM ros:humble as base

RUN useradd -m nao && echo "nao:nao" | chpasswd && adduser nao sudo

RUN apt-get update && apt-get install -y \ 
    sudo \
    python3-pip \
    git \
    ros-humble-turtle-tf2-py \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ffmpeg \
    libsm6 \
    libxext6 \
    libportaudio2 \
    imagemagick \
    alsa-base \
    alsa-utils \
    libsndfile1-dev \
    libboost-all-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0 \
    gstreamer1.0-alsa \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly

#COPY ./requirements.txt ./requirements.txt
#RUN pip3 install -r requirements.txt

RUN pip3 install numpy==1.23.5
RUN pip3 install tensorflow==2.11.0
RUN pip3 install pandas==1.5.3
RUN pip3 install PyQt5==5.14.1
RUN pip3 install motion==0.2.0
RUN pip3 install scikit-learn==1.2.2
RUN pip3 install recordtype==1.4
RUN pip3 install openai==0.27.6
RUN pip3 install sounddevice==0.4.6
RUN pip3 install pyzmq==25.0.2
RUN pip3 install qibuild 
RUN pip3 install scikit-build
RUN pip3 install toml
RUN pip3 install matplotlib
RUN pip3 install google-cloud
RUN pip3 install scikit-learn 
RUN pip3 install recordtype
RUN pip3 install google-api-core==2.11.1
RUN pip3 install google-api-python-client==2.99.0
RUN pip3 install google-auth==2.23.0
RUN pip3 install google-auth-httplib2==0.1.1
RUN pip3 install google-auth-oauthlib==0.4.6
RUN pip3 install google-cloud==0.34.0
RUN pip3 install google-cloud-speech==2.21.0
RUN pip3 install google-pasta==0.2.0
RUN pip3 install googleapis-common-protos==1.60.0
RUN pip3 install openai-whisper @ git+https://github.com/openai/whisper.git@e8622f9afc4eba139bf796c210f5c01081000472
RUN pip3 install transforms3d
RUN pip3 install Flask
RUN pip3 install flask_cors


#RUN rm requirements.txt

ENV MAIN_DIR=/home/nao
ENV CMAKE_PREFIX_PATH=/opt/ros/humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages


FROM base as build

WORKDIR ${MAIN_DIR}/src/
RUN git clone https://github.com/ros-visualization/interactive_markers.git/
WORKDIR ${MAIN_DIR}/src/interactive_markers
RUN git checkout ros2
WORKDIR ${MAIN_DIR}
RUN colcon build --packages-select interactive_markers

WORKDIR ${MAIN_DIR}/src/
RUN git clone https://github.com/ros-drivers/audio_common.git/
WORKDIR ${MAIN_DIR}/src/audio_common
RUN git checkout ros2
WORKDIR ${MAIN_DIR}
RUN colcon build --packages-select audio_common_msgs
RUN colcon build --packages-select audio_capture

FROM base

RUN usermod -aG audio nao
RUN apt-get install -y make

USER nao
COPY --from=build --chown=nao:nao ${MAIN_DIR}/install ${MAIN_DIR}/install

# Note: I don't know why, but for letter_learning_interaction, the command "source install/setup.bash"
# doesn't add the package to the AMENT_PREFIX_PATH, so we add it manually
ENV AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}:/home/nao/install/letter_learning_interaction:/home/nao/install/interface:/home/nao/install/choose_adaptive_words:/home/nao/install/nao_trajectory_following

WORKDIR ${MAIN_DIR}

CMD ["bash"]
