# Simulink-iiwa interface
The SimulinkIIWA is an interface that allows the user to control KUKA iiwa manipulaotrs from inside Simulink.
Unlike the ![KUKA Sunrise Toolbox or KST](https://github.com/Modi1987/KST-Kuka-Sunrise-Toolbox), the SimulinkIIWA interface is built upon UDP protocol instead of TCP/IP protocol.

![cover photo](https://github.com/Modi1987/Simulink-iiwa-interface/raw/master/Photos/SimulinkIIWAcover.jpg)
--------------------------------------

# Requirments:

The project in this repository was tested using:
* Matlab 2018a, running under Windows 10.
* KUKA iiwa 7R800 with Sunrise.OS 1.11.0.7

--------------------------------------

# Video tutorials:

Video tutorials on using the interface are [available in here](https://www.youtube.com/watch?v=at9xUItdidI&list=PLz558OYgHuZcK3ubmfA1rEm2UdLDDC37D).

--------------------------------------
# Installation
To setup the SimulinkIIWA interface the following steps, for Matlab and for the robot controller, shall be followed:

## On IIWA side:
* You will need a computer connected to the robot through an ethernet cable on the X66 connector of the controller.
* Open the Sunrise.Workbench on the computer side.
* Download the [SimulinkIIWA](https://github.com/Modi1987/Simulink-iiwa-interface) repository into your computer, unzip it inside a folder named "SimulinkIIWA".
* Then from inside the folder "SimulinkIIWA" pick up one of the operation modes available, for example, the [Cartesian motion open loop](https://github.com/Modi1987/Simulink-iiwa-interface/tree/master/Cartesian%20motion%20open%20loop). Each operation mode is provided in a separate folder, which contains the corresponding Sunrise.Workbench code along with the accompanying Simulink project.
* Inside the [Cartesian motion open loop](https://github.com/Modi1987/Simulink-iiwa-interface/tree/master/Cartesian%20motion%20open%20loop) folder, open the [iiwa](https://github.com/Modi1987/Simulink-iiwa-interface/tree/master/Cartesian%20motion%20open%20loop/iiwa) sub-folder. 
* You shall copy the available JAVA files into a project inside the Sunrise.Workbench. Afterwards synchronise the project to the robot controller.
* After synchronisation, the newly added application will show up in the applications-list on the smartPad of the robot under the name "SimulinkIIWADirectServoCartesian".

## On Matlab side:
* First open Matlab 2018a 
* Then, you shall install the Simulink Desktop Real-time Kernel by [following the instructions in here](https://www.mathworks.com/help/sldrt/ug/real-time-windows-target-kernel.html).
* Note: Mathworks provides the (Simulink Desktop Real-time) under Windows and Mac operating systems. Though the provided scripts were tested only under Windows. To check the installation status of the real-time kernel, type the command (rtwho) in the command window of Matlab 2018a.
--------------------------------------

# Operation:
First run the server on IIWA side using the smartPad, then you shall run the Simulink project in Matlab 2018a, as detailed in the following:

## On the IIWA side:
* Run the server application from the smartPad, you find it in the applications-list under the name "SimulinkIIWADirectServoCartesian".
* Then, you will have 10 seconds to connect to it from Simulink, if a connection was not initiated during the time limit the program will be terminated automatically.
* If the connection is terminated, you have to run the server application again from the smartPad before initiating a connection.

## On Matlab side:
* From inside the [Cartesian motion open loop](https://github.com/Modi1987/Simulink-iiwa-interface/tree/master/Cartesian%20motion%20open%20loop) folder, open the [simulink](https://github.com/Modi1987/Simulink-iiwa-interface/tree/master/Cartesian%20motion%20open%20loop/simulink) sub-folder. 
* You shall find the the Simulink project "smartDirectServoCartesian.slx", double click it. It shall open in Simulink.
* After running the "SimulinkIIWADirectServoCartesian" on the robot side from the smartPad, you shall run the Simulink project "smartDirectServoCartesian.slx" from Simulink by clicking the run button.
* The robot shall move according to the motion command from Simulink.



## Citation
Please cite the following article in your publications if it helps your research 🙏 :

```javascript
@article{doi:10.1080/0951192X.2023.2177744,
author = {M. Safeea and P. Neto},
title = {Model-based hardware in the loop control of collaborative robots: Simulink and Python based interfaces},
journal = {International Journal of Computer Integrated Manufacturing},
volume = {0},
number = {0},
pages = {1-13},
year  = {2023},
publisher = {Taylor & Francis},
doi = {10.1080/0951192X.2023.2177744},
}
```
--------------------------------------
# Update log:

* Uploaded on 3rd-March-2019
* Update on 11th-June-2019 (Bugfix for 14R820 in Cartesian motion mode).
* Update on 20th-Jan-2020 (README updated)
--------------------------------------

Copyright: Mohammad Safeea.

Provided under MIT license ([check the license](https://github.com/Modi1987/Simulink-iiwa-interface/blob/master/LICENSE)).

