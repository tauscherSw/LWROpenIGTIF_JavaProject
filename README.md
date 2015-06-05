# LWROpenIGTIF_JavaProject
Project description
===================
The software of this repository is adapted for the usage within Sunrise Workbench (KUKA Laboratories GmbH) respectively in a Sunrise project. All necessary steps on how-to-setup are described in the Setup-section.
Basically this repository represents an open source interface package for the LWR Iiwa KUKA Laboratories GmbH, which is allows to communicate via OpenIGTLink. 

Furthermore a state machine has been implemented to run on the robot controller. It's states represent different actions performed by the robot, for example impedance controlled robot-movements or gravity compensation. A bunch of states has been implemented. The operation of the state machine is realized with an external openIGTLink client. By sending commands via openIGTLink state transitions can be triggered. 

The communication (openIGTLink) is based on the following interfaces:

* Visualisation Interface: the current robot pose and the estimated force at the TCP can be transmitted to the external client
* State machine Interface: a bidirectional communication with a state control can be established to operate a state machine on the robot control. 

Setup
=====
0. Install and setup Sunrise Workbench
0. Create a Sunrise-project
0. 
Usage
=====
Software overview
===================
Contribute
==========
