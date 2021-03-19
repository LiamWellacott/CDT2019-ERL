# Controller

The controller is the decision maker, it uses abstract services provided by the other GAAN components in order to achieve high level behaviour required for the competition.

The controller's behaviour is determined by a state machine

## Running

``roslaunch gaan_controller start.launch`` will launch the controller and the other components, this is the entrypoint for the GAAN system. 

## Interfaces

As the main behavioural controller, this component makes use of all of the interfaces of the "service" components found in the other packages in this project.

inputs which can cause new behaviours are:

- receiving a summon command
- receibing a user voice command

## Limitations and Lessons Learned

- Currently only bringing is implemented, other nlp based commands need to be implemented
- The interface between nlp and controller needs work, there should be an unambiguous way to refer to objects/people in the environment. I suggest using the semantic map to achieve this.
- We need an interface component for object pose estimation which ideally would allow us to run it on a single image so it isn't constantly running. it also means we can change our approach without having to change this component.
- We believe a state machine is sufficient to control the behaviours needed for the competition, but better strategies which improve the flexibility and robustness of the solution no doubt exist.