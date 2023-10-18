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


COPY ./requirements.txt ./requirements.txt
RUN pip3 install -r requirements.txt
RUN rm requirements.txt

RUN apt-get install -y python3-pil tesseract-ocr libtesseract-dev tesseract-ocr-eng tesseract-ocr-script-latn

RUN pip3 install pytesseract tesseract-ocr
RUN pip3 install Pillow

ENV MAIN_DIR=/home/nao
ENV CMAKE_PREFIX_PATH=/opt/ros/humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages


FROM base as qi-build

ENV BOOST_ROOT_DIR=${MAIN_DIR}/boost_1_77_0 
ENV BOOST_INCLUDE_DIR=${BOOST_ROOT_DIR}/include 
ENV BOOST_LIB_DIR=${BOOST_ROOT_DIR}/lib 

RUN apt-get install -y curl

WORKDIR ${MAIN_DIR}/tmp/
RUN curl -L https://boostorg.jfrog.io/artifactory/main/release/1.77.0/source/boost_1_77_0.tar.bz2 -o boost_1_77_0.tar.bz2
RUN tar -xf boost_1_77_0.tar.bz2
WORKDIR ${MAIN_DIR}/tmp/boost_1_77_0
RUN ./bootstrap.sh && \
    ./b2 --with=all -j2 install --prefix=${BOOST_ROOT_DIR}

WORKDIR ${MAIN_DIR}/tmp/
RUN git clone https://github.com/aldebaran/libqi-python
WORKDIR ${MAIN_DIR}/tmp/libqi-python
RUN sed -i '66c\cmake_args=["-DQIPYTHON_STANDALONE:BOOL=ON", "-DBOOST_ROOT:PATH=/home/nao/boost_1_77_0", "-DBoost_INCLUDE_DIR:PATH=/home/nao/boost_1_77_0/include", "-DBoost_LIBRARY_DIR:PATH=/home/nao/boost_1_77_0/lib"],' \
    setup.py
RUN python3 ./setup.py bdist_wheel


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

COPY ./src ${MAIN_DIR}/src
RUN colcon build --packages-select interface
RUN colcon build --packages-select choose_adaptive_words
RUN colcon build --packages-select nao_trajectory_following
RUN colcon build --packages-select letter_learning_interaction
RUN colcon build --packages-select speech_recognition

# fixing png images
WORKDIR ${MAIN_DIR}
RUN mogrify ./install/choose_adaptive_words/lib/python3.10/site-packages/choose_adaptive_words/design/assets/*.png


FROM base

WORKDIR ${MAIN_DIR}
COPY --from=qi-build ${MAIN_DIR}/tmp/libqi-python/dist/*.whl ./
RUN pip3 install ./*.whl
RUN rm ./*.whl

RUN usermod -aG audio root

RUN apt-get install -y supervisor 

USER nao
RUN mkdir -p /var/log/supervisor
COPY ./docker/production/supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY --from=build --chown=nao:nao ${MAIN_DIR}/install ${MAIN_DIR}/install

# Note: I don't know why, but for letter_learning_interaction, the command "source install/setup.bash"
# doesn't add the package to the AMENT_PREFIX_PATH, so we add it manually
ENV AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}:/home/nao/install/letter_learning_interaction:/home/nao/install/interface:/home/nao/install/choose_adaptive_words:/home/nao/install/nao_trajectory_following
RUN mkdir /home/nao/strugg_letter_data
WORKDIR ${MAIN_DIR}

# launch supervisor as root
USER root
CMD ["/usr/bin/supervisord"]
