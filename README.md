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


Once the PX4 SITL is installed, create your own model of the F450 model with the files provided in mavros_off_board/urdf and mavros_off_board/sdf. The instruction and steps are explained in this thread. The steps are listed below.

1. Create a folder under Tools/sitl_gazebo/models for the F450 model called quad_f450_camera
2. Create the following files under Tools/sitl_gazebo/models/quad_f450_camera: model.config and quad_f450_camera.sdf (The sdf file and model.config is located in mavros_off_board/sdf). Additionally, create the folder meshes and urdf and add the files in mavros_off_board/urdf, mavros_off_board/meshes
3. Create a world file in Tools/sitl_gazebo/worlds called grass_pad.world (file located in mavros_off_board/worlds)
4. Create an airframe file under ROMFS/px4fmu_common/init.d-posix/airframes (This can be based off the iris or solo airframe files), give it a number (for example 1076) and name it 1076_quad_f450_camera. (You can find the airframe file at mavros_off_board/files)
5. Add the airframe file, for example 1076_quad_f450_camera to ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt at the bottom of the list starting with px4_add_romfs_files(...
6. Add the airframe name (quad_f450_camera) to the file platforms/posix/cmake/sitl_target.cmake in the command that starts with set(models … as-well as the world file grass_pad to the line starting with set(worlds...
7. Copy the models located at mavros_off_board/worlds/gazebo into .gazebo/models. In case the folder models does not exist within .gazebo, create it first.
8. Finally, add the three ROS packages to your catkin_ws and compile the project with catkin_make.

 Set Custom Takeoff Location


Change Simulation Speed
We can also set the parameters of the Gazebo,to control the speed of simulation by following 

	PX4_SIM_SPEED_FACTOR=2 make px4_sitl jmavsim
	PX4_SIM_SPEED_FACTOR=0.5 make px4_sitl jmavsim


Loading a Specific World:

	cd PX4-Autopilot
	make px4_sitl gazebo

Try in gazebo:

	param set NAV_RCL_ACT 0
	param set NAV_DLL_ACT 0
	param set CAM_RC_IN_MODE 
	pxh>make px4_sitl_default gazebo_plane_cam_grasspad

To use the following commander to start and takeoff/land:

	pxh>make px4_sitl_default gazebo_plane_cam_grasspad
	pxh>commander takeoff
	pxh>commander land

