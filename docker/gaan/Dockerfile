FROM gaan/tiago-melodic:gpu

#ENV TF_CPP_MIN_LOG_LEVEL=2

RUN pip install face_recognition
RUN apt install ros-melodic-usb-cam -y

#Adding dope
RUN cd $GAAN_WS/src && \
    source /opt/ros/melodic/setup.bash && \
    git clone -b ros-kinetic https://github.com/kini5gowda/Deep_Object_Pose dope

RUN cd $GAAN_WS/src && \
    source /opt/ros/melodic/setup.bash && \
    cd dope && \
    pip install -r requirements.txt && \
    rosdep install --from-paths src -i --rosdistro kinetic && \
    cd ../.. && \
    catkin_make

RUN pip install gdown
RUN apt install unrar

RUN cd $GAAN_WS/src/dope/weights && \
    gdown https://drive.google.com/uc?id=1rnjp2kgttraZRmqH51vW_NOaTTa6iiN2 -O weights.rar -O  weights.rar

RUN cd $GAAN_WS/src/dope/weights && \
    unrar e weights.rar

# NLP setup
# Setup for RASA-BOT
# TODO: move the virutal env somewhere else as it's not related to RASA
RUN mv /etc/apt/sources.list.d/wheezy-backports-main.list{,.bak} && \
apt update && \
apt install -y python3-dev python3-pip python3-venv

ENV OLD_PATH=$PATH
ENV RASA_ENV=/opt/venv/rasa_env
ENV SPEECH_ENV=/opt/venv/speech_env

RUN python3 -m venv $RASA_ENV
RUN python3 -m venv $SPEECH_ENV

ENV PATH="$SPEECH_ENV/bin:$OLD_PATH"

RUN pip3 install --upgrade pip

RUN cd tools && \
    git clone https://github.com/giannissc/rasa-granny-annie.git rasa-bot && \
    cd rasa-bot && \
    pip install -r requirements.txt

RUN cd tools/rasa-bot && \
    rasa train

ENV PATH="$SPEECH_ENV/bin:$OLD_PATH"

# Setup STT system
RUN apt install portaudio19-dev espeak -y && \
    pip install --upgrade pip && \
    pip install deepspeech~=0.9.3 pyaudio~=0.2.11 webrtcvad~=2.0.10 halo~=0.0.18 numpy>=1.15.1 scipy>=1.1.0 requests torch pyaudio

ENV PATH="$OLD_PATH"
RUN echo alias rasa_env="PATH=$SPEECH_ENV/bin:$OLD_PATH" >> ~/.bashrc
RUN echo alias speech_env="PATH=$SPEECH_ENV/bin:$OLD_PATH" >> ~/.bashrc
RUN echo alias deactivate="PATH=$OLD_PATH" >> ~/.bashrc

# Download the weights
RUN mkdir stt_models && cd stt_models && \
    curl -LO https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.pbmm && \
    curl -LO https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.scorer

RUN apt install -y alsa-base alsa-utils

# Build the ros workspace
RUN cd $GAAN_WS && \
    source /opt/ros/melodic/setup.bash && \
    catkin_make

EXPOSE 11345
CMD ["terminator"]
