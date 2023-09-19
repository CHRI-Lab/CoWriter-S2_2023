FROM chienpul/nao-base-lite:v3

ENV MAIN_DIR=/home/nao
ENV CATKIN_DIR=${MAIN_DIR}/catkin_ws
ENV CATKIN_SRC_DIR=${CATKIN_DIR}/src

# Bluering specific steps (if applicable)
WORKDIR ${CATKIN_SRC_DIR}
# RUN /opt/ros/noetic/bin/catkin_init_workspace

COPY ${RESOURCES_DIR}/bluering_letter_learning ./bluering_letter_learning
COPY ${RESOURCES_DIR}/choose_adaptive_words ./choose_adaptive_words

COPY ${RESOURCES_DIR}/shape_learning ./shape_learning
WORKDIR ${CATKIN_SRC_DIR}/shape_learning
RUN python3 setup.py install

# Install additional ROS dependencies
# WORKDIR ${CATKIN_SRC_DIR}
# RUN git clone https://github.com/ros-perception/image_common.git
# RUN git clone https://github.com/ros-drivers/gscam.git
# COPY ${RESOURCES_DIR}/v4l.launch ${CATKIN_SRC_DIR}/gscam/launch/v4l.launch  ---> useless if we dont use the camera
# RUN git clone https://github.com/chili-epfl/ros_markers.git
# RUN git clone https://github.com/ros-drivers/audio_common

WORKDIR ${CATKIN_DIR}
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && catkin_make

# WORKDIR ${CATKIN_SRC_DIR}
# RUN git clone https://github.com/ros-visualization/interactive_markers.git
# WORKDIR ${CATKIN_SRC_DIR}/interactive_markers
# RUN python3 ./setup.py bdist_wheel
# RUN pip3 install ./dist/*.whl 

# RUN pip3 install grpcio grpcio-tools
RUN pip3 install --upgrade pip
RUN pip3 install PyQt5
RUN pip3 install tensorflow
RUN pip3 install pandas

RUN apt-get update && apt-get install -y --no-install-recommends\
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0

# Clean
RUN apt-get clean && rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/* && rm -rf /root/.cache/*

WORKDIR ${CATKIN_DIR}
RUN chmod +x ./src/bluering_letter_learning/nodes/*
RUN ln -s /usr/bin/python3 /usr/bin/python
RUN chown -R nao:nao /home/nao/catkin_ws/src/bluering_letter_learning/
RUN chmod 755 /home/nao/catkin_ws/src/bluering_letter_learning/
RUN usermod -aG audio nao
USER nao
RUN echo "source ${CATKIN_DIR}/devel/setup.bash" >> ~/.bashrc
WORKDIR ${CATKIN_DIR}
CMD /bin/bash
