# face_rec

This component performs facial recognition given a single input picture of a persons face.

## Running

### normal execution
launch the node with 

```
rosrun face_rec face_rec.py
```
### To run demo:

put a picture of the yourself in the folder faces/known.

then run
```
    roslaunch web_cam_demo.launch
```

## Interfaces

### /gaan/face_rec

uses the current camera input and returns a list of recognised faces in the frame. To test the service run the following on the command line with the face rec node running:

```
rosservice call /gaan/face_rec "{}"
```

## Limitations and Lessons Learned

- This component works really well :)