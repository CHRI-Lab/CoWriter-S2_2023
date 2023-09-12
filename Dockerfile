FROM ros:humble

RUN useradd -m nao && echo "nao:nao" | chpasswd && adduser nao sudo
ENV MAIN_DIR=/home/nao
ENV PROJECT_DIR=${MAIN_DIR}/NAOHW-Boxjelly
USER root

RUN apt-get update && apt-get install -y sudo python3-pip git

RUN pip3 install qibuild scikit-build toml \
    matplotlib openai google-cloud scikit-learn \
    recordtype --upgrade google-api-python-client --upgrade google-cloud-speech \
    git+https://github.com/openai/whisper.git 
COPY ./requirements.txt ./requirements.txt
RUN pip3 install -r requirements.txt

WORKDIR ${PROJECT_DIR}/src/
ENV CMAKE_PREFIX_PATH=/opt/ros/humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages

COPY ./src ${PROJECT_DIR}/src
WORKDIR ${PROJECT_DIR}/src/packages
RUN git clone https://github.com/ros-visualization/interactive_markers.git/
WORKDIR ${PROJECT_DIR}/src/packages/interactive_markers
RUN git checkout humble
WORKDIR ${PROJECT_DIR}/src/
# isolation of the build environment for each module, otherwise the build remains stuck ?
RUN colcon build --packages-select choose_adaptative_words
RUN colcon build --packages-select interactive_markers
RUN colcon build --packages-select nao_trajectory_following
RUN colcon build --packages-select cowriter_letter_learning

RUN chown -R nao:nao ${PROJECT_DIR}/src/
RUN chmod 755 ${PROJECT_DIR}/src/

USER nao