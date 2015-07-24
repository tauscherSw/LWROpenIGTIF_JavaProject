# LWROpenIGTIF_JavaProject
Project description
===================
The software of this repository is adapted for the usage within Sunrise Workbench (KUKA Laboratories GmbH) respectively in a Sunrise project. All necessary steps on how-to-setup are described in the Setup-section.
Basically this repository represents an open source interface package for the LWR Iiwa 14 R820 KUKA Laboratories GmbH, which is allows to communicate via OpenIGTLink. 

Furthermore a state machine has been implemented to run on the robot controller. It's states represent different actions performed by the robot, for example impedance controlled robot-movements or gravity compensation. A bunch of states has been implemented. The operation of the state machine is realized with an external openIGTLink client. By sending commands via openIGTLink state transitions can be triggered. 

The communication (openIGTLink) is based on the following interfaces:

* Visualisation Interface: the current robot pose and the estimated force at the TCP can be transmitted to the external client
* State machine Interface: a bidirectional communication with a state control can be established to operate a state machine on the robot control. 

Setup
=====
Requirements:
-------------
* KUKA LWR iiwa 14 R820
* KUKA Sunrise Workbench (V 1.5.1.8 or later)
* FRI (fast research interface) option
* SmartServo option

Steps to setup the robot and the software
-----------------------------------------
0. Install and setup Sunrise Workbench. 
0. Create a Sunrise-project and select LBR iiwa 14 R820 as robot type. A robot application is provided by this repository, so deselect the creation of a new robot application.
0. Minimize SunriseWorkbench and copy the FRI library (<i>connectivity.fri-javadoc.zip & connectivity.fri.jar</i>) and the smart servo library (<i>roboticsAPI.smartServo-javadoc.zip & roboticsAPI.smartServo.jar</i>) in the project folder <i>KUKAJavaLib</i>
0. Go to [github/LWROpenIGTIF_JavaProject](https://github.com/tauscherSw/LWROpenIGTIF_JavaProject) and download the repository as a zip file.
0. Open the downloaded zip file and extract it's contents to the project folder. <b>The <i>src</i> folder must be overwritten!</b>.
0. Return to sunrise workspace and refresh your workspace (F5 or rigthclick (in the workspace) ->refresh). Now you should see the <i>Libs</i> folder in the project hierarchy as well as a bunch of classes inside the <i>src</i> folder, but yet with many errors.
0. Add the missing libraries to your project's build path. Rigthclick on your project and select <i>buildpath->configure build path</i>. Now go to the <i>libraries</i> section an click on <i>Add JARs</i> and select all libraries in the <i>KUKAJavaLib</i> folder of your project. Repeat this step for the Libs folder.
0. Now all errors should be solved, after your project was rebuilt.
0. Configure the safety configuration of your project as desired.
0. Open the file <i>StationSetup.cat</i> in your project and open the <i>Software</i> section. Select for installation:
 * <i>Direct Servo Motion Extension</i>
 * <i>Fast Robot Interface</i>
 * <i>Smart Servo Motion Extension</i>
0. Install the Sunrise-project on the robot controller.
0. Create a tool template in your workspace and name it: <i>openIGTLProjectTool</i>. Add your specific load data and your tcp. If no tool is mounted on your robot's flange do not modify the new tool's data.
0. Synchronize your sunrise project

Usage
=====

Software overview
===================

Contribute
==========
