# Navigation

The navigation component wraps the tiago simulation packages to provide basic navigation actions needed for the competition. 

The tiago simulation packages implement the ROS navigation stack. A map must be pregenerated in order for the simulation software to work (e.g. by running the `nav_ralt_tiago.launch` with state argument set to mapping and using teleop). This is because of how the launch files initialise the tiago simulation, you might be able to figure out a way around this...

## Running

To run the navigation component make sure the navigation stack from the tiago simulation has been launched (e.g. with `nav_ralt_tiago.launch` in `sim_launch`).

## Interfaces

The navigation component provides the following services to the rest of the system

### /gaan/navigation/navigate_to

A goal pose is given as input, the navigation component requests the tiago navigation stack to move the robot to this pose. 

## Limitations and Lessons Learned

- The component probably should be implemented as an action server
- A static semantic map is currently being used, we need to implement this part to be competitive
- Related to the above, we don't do SLAM, we precreate a map. It would be good to have the set up phase include some SLAM component which also links observed objects in a semantic map
- Other high level services may be useful for other tasks types: Guiding and Following 
- see my issue breakdown #25 on the github for suggested roadmap for this component