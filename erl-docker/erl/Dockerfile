FROM erl/tiago-melodic:gpu


RUN pip install face_recognition
RUN apt install ros-melodic-usb-cam -y

EXPOSE 11345
CMD ["terminator"]
