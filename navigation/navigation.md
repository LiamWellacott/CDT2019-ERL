# Navigation

The navigation module wraps the tiago simulation packages to provide basic navigation actions needed for the competition. To run the navigation module make sure the navigation stack from the tiago simulation has been launched (e.g. with `nav_ralt_tiago.launch` in `hello_tiago`).

The tiago simulation packages implement the ROS navigation stack. A map must be pregenerated in order for the simulation software to work (e.g. by running the `nav_ralt_tiago.launch` with state argument set to mapping and using teleop). This is because of how the launch files initialise the tiago simulation, you might be able to figure out a way around this...

## Services

The navigation module provides the following services to the rest of the system

### NavigateTo

A goal pose is given as input, the navigation module requests the tiago navigation stack to move the robot to this pose. 

## TODO

- The module probably should be an action server, but I spent too much time struggling with basic things to fix it.