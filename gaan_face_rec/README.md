# To run demo:

put a picture of the known persons in the folder faces/known.

then run
```
    roslaunch web_cam_demo.launch
```

# Servics

``/erl/face_rec`` uses the current camera input and returns a list of recognised faces in the frame. To test the service run the following on the command line with the face rec node running:

```
rosservice call /erl/face_rec "{}"
```
