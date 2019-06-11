# Simulink-iiwa interface
The Simulink-iiwa is an interface that allows the user to control KUKA iiwa manipulaotrs from Simulink.
Unlike the ![KUKA Sunrise Toolbox or KST](https://github.com/Modi1987/KST-Kuka-Sunrise-Toolbox), the Simulink interface is built upon UDP protocol instead of TCP/IP protocol.

--------------------------------------

# Requirments:

The project in this rpository was tested using:
* Matlab 2018a, running under Windows 10.
* KUKA iiwa 7R800 with Sunrise.OS 1.11.0.7

--------------------------------------

# Video tutorials:

Video tutorials on using the interface are [available in here](https://www.youtube.com/watch?v=at9xUItdidI&list=PLz558OYgHuZcK3ubmfA1rEm2UdLDDC37D).

--------------------------------------
# Installation
There are various steps that you shall dow on the Matlab side and on the iiwa side:

## Matlab side:
* First open Matlab 2018a 
* Then, you shall install the Simulink Desktop Real-time Kernel by ![following the instructions in here](https://www.mathworks.com/help/sldrt/ug/real-time-windows-target-kernel.html).
* Note: Mathworks provides the (Simulink Desktop Real-time) under Windows and Mac operating systems. Though the provided scripts were tested only under Windows. To check the installation status of the kernel type (rtwho) in the command window of Matlab 2018a. 

## IIWA side:
* You will need a computer connected to the robot through an ethernet cable on X66 connector.
* Open the Sunrise.Workbench on the computer side.
* Download the repository into your computer, unzip it inside a folder (SimulinkIIWA) for example.
* Then from inside the (SimulinkIIWA) pick up one of the operation modes available, for example (Cartesian motion open loop). Each operation mode is provided in separate folder, which contains the corresponding Sunrise.Workbench code along with the Simulink interface.
* Inside the (Cartesian motion open loop) folder, open the (iiwa) sub-folder. 
* You shall copy the available (java) file into a project inside the Sunrise.Workbench then synchronise into the robot controller.
* The newly added application shall show up in the application list on the smart pad of the robot.
--------------------------------------

# Operation:
First run the server on IIWA side using the smart pad, then you shall run the Simulink project in Matlab 2018a.

## IIWA side:
* Run the server application from the smart pad, you find it in the applications list under the name (SimulinkIIWADirectServoCartesian).
* You will have 10 seconds to connect, if a connection was not initiated during the time limit the program will be terminated automatically.
* If the connection is terminated, you have to run the server again before initiating a connection.

## Matlab side:
* Inside the (Cartesian motion open loop) folder, open the (iiwa) sub-folder. 
* You shall find the simulink application, run it using Matlab 2018a.
* After runnng the (SimulinkIIWADirectServoCartesian) on the robot side, you shall run the simulink project from matlab.
* The robot shall move according to the motion command from simulink.

--------------------------------------
# Update log:

* Uploaded on 3rd-March-2019
* Update on 11th-June-2019 (Bugfix for 14R820 in Cartesian motion mode).

--------------------------------------

Copyright: Mohammad Safeea.

The project is a work in progress, use it under your own responsibility, check the license.

