FROM ros:humble

# CREATING USERS
RUN useradd -m nao && echo "nao:nao" | chpasswd && adduser nao sudo
ENV MAIN_DIR=/home/nao

USER root

RUN apt-get update && apt-get install -y python2 python3-pip python3-tk wget nano sudo
RUN pip3 install matplotlib flask scikit-learn flask-cors openai SpeechRecognition recordtype

ENV PROJECT_DIR=${MAIN_DIR}/NAOHW-RedBack
COPY ./resources/NAOHW-RedBack ${MAIN_DIR}/NAOHW-RedBack

WORKDIR ${PROJECT_DIR}/src/nao_ros2_ws
ENV CMAKE_PREFIX_PATH=/opt/ros/humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages

#RUN . /opt/ros/humble/setup.bash
RUN colcon build
RUN echo "source ${PROJECT_DIR}/src/nao_ros2_ws/install/setup.bash" >> ~/.bashrc

ENV NAOQI_DIR=${MAIN_DIR}/naoqi
WORKDIR ${NAOQI_DIR}
RUN wget https://community-static.aldebaran.com/resources/2.8.6/pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327.tar.gz
RUN tar -xvzf pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327.tar.gz
ENV PYTHONPATH=${PYTHONPATH}:${NAOQI_DIR}/pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327/lib/python2.7/site-packages

RUN chown -R nao:nao ${PROJECT_DIR}/src/nao_ros2_ws
RUN chmod 755 ${PROJECT_DIR}/src/nao_ros2_ws

WORKDIR ${PROJECT_DIR}
USER nao
RUN echo "source ${PROJECT_DIR}/src/nao_ros2_ws/install/setup.bash" >> ~/.bashrc
CMD /bin/bash
