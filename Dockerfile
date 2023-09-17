FROM ros:humble

RUN useradd -m nao && echo "nao:nao" | chpasswd && adduser nao sudo
ENV MAIN_DIR=/home/nao
ENV PROJECT_DIR=${MAIN_DIR}/NAOHW-Boxjelly
USER root

RUN apt-get update && apt-get install -y sudo python3-pip git
RUN apt-get install -y ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations
RUN apt-get install  -y ffmpeg libsm6 libxext6
RUN sudo apt-get install libportaudio2


RUN pip3 install qibuild scikit-build toml \
    matplotlib openai google-cloud scikit-learn \
    recordtype --upgrade google-api-python-client --upgrade google-cloud-speech \
    git+https://github.com/openai/whisper.git 
COPY ./requirements.txt ./requirements.txt
RUN pip3 install -r requirements.txt
RUN pip3 install transforms3d

WORKDIR ${PROJECT_DIR}/src/
ENV CMAKE_PREFIX_PATH=/opt/ros/humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages

COPY ./src ${PROJECT_DIR}/src
WORKDIR ${PROJECT_DIR}/src
RUN git clone https://github.com/ros-visualization/interactive_markers.git/
WORKDIR ${PROJECT_DIR}/src/interactive_markers
RUN git checkout ros2
WORKDIR ${PROJECT_DIR}
# isolation of the build environment for each module, otherwise the build remains stuck ?
RUN colcon build --packages-select interface
RUN colcon build --packages-select interactive_markers
RUN colcon build --packages-select choose_adaptive_words
RUN colcon build --packages-select nao_trajectory_following
RUN colcon build --packages-select letter_learning_interaction

RUN chown -R nao:nao ${PROJECT_DIR}/src/
RUN chmod 755 ${PROJECT_DIR}/src/

USER nao