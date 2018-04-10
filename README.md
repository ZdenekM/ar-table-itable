# ARTable - the main repository

[![Build Status](https://travis-ci.org/robofit/artable.svg)](https://travis-ci.org/robofit/artable)

ARTable - vision of a near future workspace, where human and robot may safely and effectively collaborate. Our main focus is on human-robot interaction and especially on robot programming - to make it feasible for any ordinary skilled worker. The interaction is based mainly on interactive spatial augmented reality - combination of projection and touch sensitive surface. However, more modalities are integrated or currently under development.

### Repositories / packages

This repository holds the main components of the system, which are not specific for any particular setup (combination and type of components) or robot:

 * [art_brain](https://github.com/robofit/artable/tree/master/art_brain) - decision making
 * [art_bringup](https://github.com/robofit/artable/tree/master/art_bringup) - to launch the system
 * [art_calibration](https://github.com/robofit/artable/tree/master/art_calibration) - AR marker-based calibration of cameras.
 * [art_db](https://github.com/robofit/artable/tree/master/art_db) - permanent storage for object types, programs, etc.
 * [art_projected_gui](https://github.com/robofit/artable/tree/master/art_projected_gui) - shows system state, allows to set program parameters, etc-
 * [art_simple_tracker](https://github.com/robofit/artable/tree/master/art_simple_tracker) - not a real tracker, it "tracks" objects based on already assigned IDs and performs position/orientation filtering from multiple detectors
 * [art_sound](https://github.com/robofit/artable/tree/master/art_sound) - a sound interface: plays sound for selected system events (error).
 * [art_table_pointing](https://github.com/robofit/artable/tree/master/art_table_pointing) - uses Kinect skeleton tracking to compute where user points on the table.
 * [art_touch_driver](https://github.com/robofit/artable/tree/master/art_touch_driver) - reads data from touch foil (which is HID device) a publishes it as ROS messages.

There is also repository with [ROS messages](https://github.com/robofit/artable-msgs) and [various utilities](https://github.com/robofit/artable-utils).

For each integrated robot, there are two repositories: one with custom packages providing high-level functions compatible with ARTable ROS API and one with implementation of art_brain plugin (```-interface``` one):

* PR2
  * [https://github.com/robofit/artable-pr2](https://github.com/robofit/artable-pr2)
  * [https://github.com/robofit/artable-pr2-interface](https://github.com/robofit/artable-pr2-interface)
* DOBOT Magician
  * [https://github.com/robofit/artable-dobot](https://github.com/robofit/artable-dobot)
  * [https://github.com/robofit/artable-dobot-interface](https://github.com/robofit/artable-dobot-interface)

Currently supported setups (see links for further information):

 * [ARTable setup 1](https://github.com/robofit/artable-setup-1)
 * [ARTable setup 2](https://github.com/robofit/artable-setup-2)
 * [ARTable setup 3](https://github.com/robofit/artable-setup-3)
 
 Any supported setup may be used with any supported robot (or even without one).

### Functionality

The system has two main modes: setting program parameters and program execution.

The (slightly outdated) video below shows how operator uses touch sensitive table and augmented reality interface to set parameters of a program consisting of two pick and place tasks. For each task, the operator sets object type to be picked up, area from which objects of that type should be picked and location on the table where the robot (PR2) should place them.

[![ARTable video](https://i.ytimg.com/vi/M_KxpIJo1LA/0.jpg)](https://youtu.be/M_KxpIJo1LA)

Currently, the robot program has to be created beforehand (e.g. using script like [this](https://github.com/robofit/artable/blob/master/art_db/scripts/simple_trolley.py). Then, program parameters could be easily set using the projected interface - to make it as simple as possible, the system is based on complex instructions, with high-level of abstraction (for supported instructions see [art_msgs/ProgramItem.msg](https://github.com/robofit/artable-msgs/blob/master/art_msgs/msg/ProgramItem.msg)).

### API

All topics, parameters and services can be found in `/art` namespace.

TBD

### Installation

TBD

### Contributing

 - Follow [PyStyleGuide](http://wiki.ros.org/PyStyleGuide) or [CppStyleGuide](http://wiki.ros.org/CppStyleGuide)
 - Use [catkin_lint](http://fkie.github.io/catkin_lint/) to check for common problems (```catkin_lint -W2 your_package_name```)
 - Use [roslint](http://wiki.ros.org/roslint) to run static analysis of your code.
 - Ideally, create and use unit tests.
 - Feel free to open pull requests!

### Publications

 * MATERNA Zdeněk, KAPINUS Michal, BERAN Vítězslav a SMRŽ Pavel. Using Persona, Scenario, and Use Case to Develop a Human-Robot Augmented Reality Collaborative Workspace. In: HRI 2017. Vídeň: Association for Computing Machinery, 2017, s. 1-2. ISBN 978-1-4503-4885-0.
 * MATERNA Zdeněk, KAPINUS Michal, ŠPANĚL Michal, BERAN Vítězslav a SMRŽ Pavel. Simplified Industrial Robot Programming: Effects of Errors on Multimodal Interaction in WoZ experiment. In: Robot and Human Interactive Communication (RO-MAN). New York City: Institute of Electrical and Electronics Engineers, 2016, s. 1-6. ISBN 978-1-5090-3929-6.
