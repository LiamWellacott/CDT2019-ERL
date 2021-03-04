FROM gaan/tiago-melodic:gpu

RUN pip install face_recognition
RUN apt install ros-melodic-usb-cam -y

# TODO remove these?
RUN cd tools && \
    git clone https://github.com/johnwheeler/flask-ask.git && \
    cd flask-ask && \
    pip install -r requirements-dev.txt && \
    python setup.py install

RUN cd tools && \
    wget https://bin.equinox.io/c/4VmDzA7iaHb/ngrok-stable-linux-amd64.zip && \
    unzip ngrok-stable-linux-amd64.zip && rm ngrok-stable-linux-amd64.zip

# Build the ros workspace
RUN cd $GAAN_WS && \
    source /opt/ros/melodic/setup.bash && \
    catkin_make

EXPOSE 11345
CMD ["terminator"]
