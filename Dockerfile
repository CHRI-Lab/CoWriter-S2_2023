FROM aplaire/nao-base:boxjelly

ENV MAIN_DIR=/home/nao 
ENV PROJECT_DIR=${MAIN_DIR}/NAOHW-Boxjelly

# specific boxjelly
WORKDIR ${MAIN_DIR}

COPY ./resources/NAOHW-Boxjelly ./NAOHW-Boxjelly

WORKDIR ${PROJECT_DIR}
RUN pip3 install -r requirements.txt

RUN rm -rf build/*
RUN rm -rf devel/*

RUN /opt/ros/noetic/bin/catkin_make --only-pkg-with-deps chilitags_catkin
RUN /opt/ros/noetic/bin/catkin_make --only-pkg-with-deps ros_markers
RUN /opt/ros/noetic/bin/catkin_make -DCATKIN_WHITELIST_PACKAGES=""

RUN apt-get install -y portaudio19-dev nano

RUN git clone https://github.com/ros-visualization/interactive_markers.git
WORKDIR ${PROJECT_DIR}/interactive_markers
RUN python3 ./setup.py bdist_wheel
RUN pip3 install ./dist/*.whl 

# Clean
RUN apt-get clean && rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/* && rm -rf /root/.cache/*

COPY ./resources/NAOHW-Boxjelly/share ./NAOHW-Boxjelly/share

USER nao
RUN echo "source ${PROJECT_DIR}/devel/setup.bash" >> ~/.bashrc
WORKDIR ${PROJECT_DIR}
CMD /bin/bash
