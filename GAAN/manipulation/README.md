# manipulation

This component abstracts all control of the torso and arm to support the manipulation of objects in the environment.

## Running

You can launch the component and the downstream manipulation topics with:

```
roslaunch manipulation start_manipulation.launch
```

In addition, you can run the following to launch a simulation environment suited for testing the manipulation topics

```
roslaunch manipulation manipulation_demo.launch
```

## Interfaces

### /gaan/manipulate

The component provides this single service for all manipulation types. The service has an "action" parameter which determines what the manipulation server will do with the request, and a pose which is used differently based on the action. The possible actions are:

- PICK (grasp object at pose)
- PLACE (place object at pose)
- PRESENT (fixed motion, holds object directly in front of Tiago and drops it)

## Limitations and Lessons Learned

- The manipulation component is built on top Tiago demo software ``pick_and_place_server``. This server calculates grasp poses (generates a unit sphere, no object affordance analysis), and creates a goal for ``moveit`` (ROS manipulation stack). We found this rarely worked. Overall the manipulation component is not functioning enough to be used in its current state.
- We suggest redoing this component from first principles.
