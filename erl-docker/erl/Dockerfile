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

#Adding dope    
RUN cd $ERL_WS/src && \
    git clone -b ros-kinetic https://github.com/NVlabs/Deep_Object_Pose.git dope && \
    cd dope && \
    pip install -r requirements.txt && \
    rosdep install --from-paths src -i --rosdistro kinetic && \
    cd ../.. && \
    catkin_make && \
    mkdir weights && \
    cd weights && \
    wget "https://drive.google.com/file/d/1rnjp2kgttraZRmqH51vW_NOaTTa6iiN2/view?usp=sharing" && \
    unrar e weights.rar
    
    

EXPOSE 11345
CMD ["terminator"]
