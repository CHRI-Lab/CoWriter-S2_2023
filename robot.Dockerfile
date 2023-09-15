FROM python:2.7-stretch

RUN useradd -m nao && echo "nao:nao" | chpasswd && adduser nao sudo
ENV MAIN_DIR=/home/nao

USER root

ENV NAOQI_DIR=${MAIN_DIR}/naoqi
WORKDIR ${NAOQI_DIR}
RUN wget https://community-static.aldebaran.com/resources/2.8.6/pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327.tar.gz
RUN tar -xvzf pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327.tar.gz
ENV PYTHONPATH=${PYTHONPATH}:${NAOQI_DIR}/pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327/lib/python2.7/site-packages

RUN pip install flask

COPY ./src/controller/nao_controller.py ${MAIN_DIR}/controller/nao_controller.py

RUN chown -R nao:nao ${MAIN_DIR}
RUN chmod 755 ${MAIN_DIR}

WORKDIR ${MAIN_DIR}
USER nao
CMD /bin/bash