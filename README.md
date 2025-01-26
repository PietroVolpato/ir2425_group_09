# Assignment 2

GROUP 09

Filippo D'Emilio filippo.demilio@studenti.unipd.it <br>
Pietro Volpato, pietro.volpato@studenti.unipd.it <br>
Stefano Deriu, stefano.deriu@studenti.unipd.it <br>

In this repository there is only the package of our solution for the task.
You need to clone also the packages tiago_iaslab_simulation and gazebo_ros_link_attacher inside your workspace.

### HOW TO RUN ###

1) Clone the package ir2425_group_09 into your workspace and build.

2) Open 3 terminals and run the following commands:

    - TERMINAL 1: roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment2

    - TERMINAL 2: roslaunch ir2425_group_09 setup.launch

    - TERMINAL 3: roslaunch ir2425_group_09 launcher.launch

3) Different runs of the code may vary the Moveit trajectory outcomes, this might result in some unexpected behaviours

**IMPORTANT**:<br>
Terminal 3 will provide constant feedback during the execution of the task.<br>
You might need to wait until Tiago set up in the gazebo simulation: If you run the command in terminal 3<br>
too early, Tiago's arm might still be moving to initial configuration and will result in a collision with a<br>
wall. If this happens, the nodes will shut down (in order to avoid unexpected behaviours)
