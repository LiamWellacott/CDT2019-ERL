FROM erl/tiago-melodic:gpu


RUN pip install face_recognition
RUN apt install ros-melodic-usb-cam -y

#RUN apt install -y git-core curl build-essential openssl libssl-dev \
#    && git clone https://github.com/nodejs/node.git \
#    && cd node \
#    && ./configure \
#    && make \
#    && make install
#RUN apt install -y ros-melodic-rosbridge-server

RUN cd tools && \
    git clone https://github.com/johnwheeler/flask-ask.git && \
    cd flask-ask && \
    pip install -r requirements-dev.txt && \
    python setup.py install

EXPOSE 11345
CMD ["terminator"]
