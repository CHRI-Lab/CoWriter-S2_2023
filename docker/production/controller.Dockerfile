FROM python:2.7-stretch

RUN useradd -m nao && echo "nao:nao" | chpasswd && adduser nao sudo
ENV MAIN_DIR=/home/nao

USER root

RUN pip install flask

ENV NAOQI_DIR=${MAIN_DIR}/naoqi
WORKDIR ${NAOQI_DIR}
RUN wget https://community-static.aldebaran.com/resources/2.8.6/pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327.tar.gz
RUN tar -xvzf pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327.tar.gz
ENV PYTHONPATH=${PYTHONPATH}:${NAOQI_DIR}/pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327/lib/python2.7/site-packages

USER nao
COPY ./src/controller/ ${MAIN_DIR}/controller/
WORKDIR ${MAIN_DIR}/controller
CMD ["python2", "controller.py"]