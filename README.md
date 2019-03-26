# hsr_knowledge

This is the knowledge implementation of the SUTURO1819 project.

Use wstool in a seperate workspace to install the dependencies from *dependencies.rosinstall*, including *iai_common_messages* and *knowrob*.

Run the Prolog interpreter with `rosrun rosprolog rosprolog object_state`.

## Setup

Create one catkin workspace for the dependencies and load the wstool file into the src directory

```
# to setup basic catkin workspace
source /opt/ros/kinetic/setup.bash
rosdep update
mkdir -p knowledge_deps/src
cd knowledge_deps
catkin build 
# would recommend build over catkin_make. install catkin_tools via apt if needed

# now load the dependencies.rosinstall into your wstool config 
cd src
wstool init
wstool merge https://raw.githubusercontent.com/Suturo1819/hsr_knowledge/master/dependencies.rosinstall
wstool update
rosdep install --ignore-src --from-paths .
cd ..
catkin build
```

The build will take some time, and if it fails, use `source devel/setup.bash` to get the current build. The run `catkin build` again.

Now, in another workspace you can load this repository.

```
source /path/to/knowledge_deps/devel/setup.bash
cd /path/to/all/the/workspaces # it's /home/suturo/robocup in our case
mkdir -p knowledge_ws/src
cd knowledge_ws
catkin build

#clone this repo
cd src
git clone https://github.com/Suturo1819/hsr_knowledge.git
cd ..
catkin build
source devel/setup.bash 
```

You should now have two workspaces, one named `knowledge_deps` and the other one `knowledge_ws`. You will probably never need to build the `_deps` workspace again, but keep updating `hsr_knowledge` in the `_ws` workspace occasionally and use `catkin build` to get the new functionality into your environment.

Don't forget to put the `source` commands with absolute path into your `.bashrc` if your want to source the workspace in each new terminal. Remeber also, that the last `source` command in your `.bashrc` always overwrites all the previous ones. Look into [workspace overlaying](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying) for further explanation.

## Usage

Launch the robot, since the object state ublisher needs a map frame. The hsr can be in standby though.

`roslaunch object_state object_state.launch` launches the beliefstate of the hsr objects, the object_state_publisher and the perception subscriber. The subscriber listens to the topic `/hsr_perception/results`, decodes the message and asserts it into the beliefstate.

Through json_prolog queries you can obtain the data:

`object_at_table(Instance).` provides all objects at the approximate position of the table (-1, 1, 0.8).

`object_of_type(knowrob:'Cup', Instance).` gives all objects of type Cup. Replace Cup with your favourite class.

A more adventurous predicate can be useful to find objects at different locations:

`object_at(knowrob:'Cup', ['map', _, [-1,1,0.8],[0,0,0,1]], 0.4, Instance).` This will give you all the instances of type Cup within a 0.4 meter distance from (-1, 1, 0.8). Make the first argument a wildcard `_` if you don't care about the type.

## Protege for Ontology

Use the ROS-Protege version from Daniel, to load ontologies, that reference ros-packages. https://github.com/Suturo1819/ros-protege.git
