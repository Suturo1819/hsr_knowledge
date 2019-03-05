# hsr_knowledge

This is the knowledge implementation of the SUTURO1819 project.

Use wstool in a seperate workspace to install the dependencies from *dependencies.rosinstall*, including *iai_common_messages* and *knowrob*.

Run the Prolog interpreter with `rosrun rosprolog rosprolog object_state`.

## Usage

Launch the robot, since the object state ublisher needs a map frame. The hsr can be in standby though.

`roslaunch object_state object_state.launch` launches the beliefstate of the hsr objects, the object_state_publisher and the perception subscriber. The subscriber listens to the topic `/hsr_perception/results`, decodes the message and asserts it into the beliefstate.

Through json_prolog queries you can obtain the data:

`object_at_table(Instance).` provides all objects at the approximate position of the table (-1, 1, 0.8).

`object_of_type(knowrob:'Cup', Instance).` gives all objects of type Cup. Replace Cup with your favourite class.

A more adventurous predicate can be useful to find objects at different locations:

`object_at(knowrob:'Cup', ['map', _, [-1,1,0.8],[0,0,0,1]], 0.4, Instance).` This will give you all the instances of type Cup within a 0.4 meter distance from (-1, 1, 0.8). Make the first argument a wildcard `_` if you don't care about the type.

## Ontology

Use the ROS-Protege version from Daniel, to load ontologies, that reference ros-packages. https://github.com/Suturo1819/ros-protege.git
