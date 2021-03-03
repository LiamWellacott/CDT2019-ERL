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

RUN cd tools && \
    wget https://bin.equinox.io/c/4VmDzA7iaHb/ngrok-stable-linux-amd64.zip && \
    unzip ngrok-stable-linux-amd64.zip && rm ngrok-stable-linux-amd64.zip

EXPOSE 11345
CMD ["terminator"]
