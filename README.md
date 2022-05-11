# CSCI5551proj
Group Number: 12
Group topic: Autonomous Landing of a UAV
Member: Junsen Liao,
	Zhouyang Gao,
	Fengyun Shi

The operative system we used is ubuntu 20.04.4 LTS(Focal Fossa) with ROS Melodic and Gazebo.
We created a Oracle VM virtualBOx to set up the ubuntu 20.04.4 LTS.
To create the basic environment for UAV, we need several steps before the simulation.
First, we need to install ROS Melodic by following the ROS.org documentation.
Second, we need to download the PX4-Autopilot file using following command because PX4 supports Software In the Loop simulation(SITL).
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
We had to run the following command, to set up development environment that includes Gazebo and jMAVSim simulators,one single bash can do it.
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
We also met some problems and errors in set up stage, like the memory problem and package location not found.
Once we made these changes on coumputer, we have to restart it on completion.
Third,In oder to enable communication between computers and PX4 flight stack,We choose binary installation instead of source installation on ubuntu.
sudo apt-get install ros-Melodic-mavros ros-Melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
To test the simulation on Gazebo, we can run following code.
cd /path/to/PX4-Autopilot
make px4_sitl gazebo

Finally,we need to do model configuration part for the file.

For the test part,
on the one part,

on the other part,
 Set Custom Takeoff Location


Change Simulation Speed
We can also set the parameters of the Gazebo,to control the speed of simulation by following 
PX4_SIM_SPEED_FACTOR=2 make px4_sitl jmavsim
PX4_SIM_SPEED_FACTOR=0.5 make px4_sitl jmavsim


Loading a Specific World
cd PX4-Autopilot
make px4_sitl gazebo

Try in gazebo 
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
param set CAM_RC_IN_MODE 1

pxh>make px4_sitl_default gazebo_plane_cam_grasspad
pxh>commander takeoff
pxh>commander land

