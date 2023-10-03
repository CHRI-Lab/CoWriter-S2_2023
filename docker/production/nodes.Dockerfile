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
    libsndfile1-dev


COPY ./requirements.txt ./requirements.txt
RUN pip3 install -r requirements.txt
RUN rm requirements.txt

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

COPY ./src ${MAIN_DIR}/src
RUN colcon build --packages-select interface
RUN colcon build --packages-select choose_adaptive_words
RUN colcon build --packages-select nao_trajectory_following
RUN colcon build --packages-select letter_learning_interaction

# fixing png images
WORKDIR ${MAIN_DIR}
RUN mogrify ./install/choose_adaptive_words/lib/python3.10/site-packages/choose_adaptive_words/design/assets/*.png


FROM base

RUN usermod -aG audio root

RUN apt-get install -y supervisor 

USER nao
RUN mkdir -p /var/log/supervisor
COPY ./docker/production/supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY --from=build --chown=nao:nao ${MAIN_DIR}/install ${MAIN_DIR}/install
COPY ./share ${MAIN_DIR}/share

# Note: I don't know why, but for letter_learning_interaction, the command "source install/setup.bash"
# doesn't add the package to the AMENT_PREFIX_PATH, so we add it manually
ENV AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}:/home/nao/install/letter_learning_interaction:/home/nao/install/interface:/home/nao/install/choose_adaptive_words:/home/nao/install/nao_trajectory_following

WORKDIR ${MAIN_DIR}

# launch supervisor as root
USER root
CMD ["/usr/bin/supervisord"]