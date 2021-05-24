# storeplanner

## Description
This repository contains ROS Code to manage planning in retail stores.
## Usage
### TIAGo-base
* Follow the official installation of the robot [here](http://wiki.ros.org/Robots/PMB-2/Tutorials/Installation/PMB2Simulation) and install it into the ROS workspace.
* Clone this repository in the same workspace and build both
* Comment and add/modify the line as shown
```bash
<!--  <xacro:include filename="$(find pmb2_description)/urdf/base/base.urdf.xacro"/> -->
<xacro:include filename="$(find storeplanner)/robot_description/base.urdf.xacro"/>
```
in the file 
```bash
.../pmb2_public_ws/src/pmb2_robot/pmb2_description/urdf/base/base_sensors.urdf.xacro
```
This enables to use our robot model consisting into the TIAGo-base + 4 additional RGBD cameras mounted on top.
* modify as you please the tolerances for the final pose of the robot on a given goal: 

```bash
# GoalTolerance
  xy_goal_tolerance: 0.2
  yaw_goal_tolerance: 0.2
```
in the file located at
```bash
.../pmb2_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_pmb2/config/base/teb/
```
To have more flexibility on the robot, I suggest to set ```yaw_goal_tolerance: 6.28``` so any yaw is allowed.

* Use the following command to spawn the robot within the store map
```bash
roslaunch storeplanner visual.launch
```

### Setting up the world model
Given the store planimetry, to enable the simulation of the store, we need a Gazebo model. Blender could be used for this step.
After generating a Blender model, export it as .dae file under the path
```bash
~/catkin_ws/src/storeplanner/models/<store_name>/meshes/model.dae
```
Eventually, if textures are used to complete the model, they have to be placed inside the same folder of the .dae model.
Another store information that should be stored is a file containing the metadata about the store shelves
```bash
~/catkin_ws/src/storeplanner/models/<store_name>/shelves/shelves.json
```
An example of this file is given below
```json
{
  "shelves" : [
    {
      "id" : "1", 
      "x" : 26.5,
      "y" : 11.45,
      "z" : 1.65,
      "w" : 0.7,
      "h" : 8.15
    },
    {
      "id" : "2",
      "x" : 25.8,
      "y" : 22.4,
      "z" : 2.0,
      "w" : 1.2,
      "h" : 8.15
    },
    ...
    {
      "id" : "32",
      "x" : 3.05,
      "y" : 15.8,
      "z" : 2.0,
      "w" : 7.6,
      "h" : 1.5
    }
  ]
}
```
### Mapping (classic step)
The mapping step uses gmapping with the configurations of Tiago-Base.
* Launch the mapping step: spawn robot and the store
```bash
roslaunch storeplanner map_store.launch public_sim:=true
```
* Then move the robot around and explore all the store by typing the following in a new terminal
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=nav_vel
```
I prefer this solution with respect to the classic key_teleop. To install this package simply run
```bash
sudo apt install ros-melodic-teleop-twist-keyboard
```
* Once done, to save the map in the right folder run
```bash
rosservice call /pal_map_manager/save_map "directory: ''"
```
This will save the map in the hidden folder .pal at home level. To save it where is more needed (inside the folder containing all the maps) run
```bash
rosservice call /pal_map_manager/save_map "directory: '../../../catkin_ws/src/storeplanner/maps/<store_name>/<map_name>'"
```

### Mapping (custom)
Given that we have a perfectly known map of the store we should use this information.
First of all you need to generate a .dae file for the world. Place this file under ```/storeplanner/models/<store_name>/meshes/``` and create/modify the world under ```/storeplanner/worlds/```. Launch files have to be changed accordingly.
**Remarks** 
1. It is good practise to have a collision all over the boundaries of the store. This avoids the possibility for the global planner to plan a path for the desired waypoint which is outside the store (it can happen only if you map with the classic step though). 

2. Given 1, if the robot is spawn at (0,0) as well as the map (by deafault) they will collide. As a workaround, simply shift the world of a desired offset by the parameter ```<pose>-5.0 -5.0 0.0 0.0 0.0 0.0</pose>``` inside the world file. This however modifies only the gazebo part and does not shift the map for rviz that uses the package ```map_server```. To have coherency, after generating the custom map as will be explained next, in the .yaml file you have to also shift the pose of the same amount.

#### Create a map
In retail the planimetry is known. Since we are assuming to have a world model from the previous step, done e.g. in Blender, it is possible to extract a top-view image corresponding to the dae model. It is suggested to perform some simple image processing on the map and render it such to have a 2-valued map (white -> completely free cell, black -> completely occupied cell), since in this case the map is deterministic (at least for the static elements of the store).
Save it under the right directory as ```/storeplanner/maps/<store_name>/map.pgm``` and ```/storeplanner/maps/<store_name>/submap_0.pgm```. The other files that are required to have compability with the Pal Robotics packages can be copied directly from another map. The only file to be modified is the map.yaml:
```bash
image: submap_0.pgm
resolution: //must be calculated by hand from the image
origin: [//x_shift as Remark 2., //y_shift as Remark 2., 0.000000]
negate: 0
occupied_thresh: 0.65 //not used since not probabilistic map (leave it as it is)
free_thresh: 0.196 //not used since not probabilistic map (leave it as it is)
```
Once the map is created, we can set up the .json file containing the shelfs coordinates and the .json file containing all the desidred crossroads for **Task 2**.
Type the following on the command line
```bash
cd <your_workspace>/src/storeplanner/scripts/
python2 map_details.py
```
Required params for this script are the `store_name` and `map_name` that correspond to the folder names.

This will enable the generation of shelves for the specified store and map together with the free spots to perform a planned itinerary.

### Navigation (Task 1)
To perform **Task 1** it is required to have generated the .json file containing the shelves coorinates.

The navigation step
* Launch the navigation step: spawn robot and the store together with the map already created
```bash
roslaunch storeplanner nav_store.launch 
```
The lost argument of the tutorial can be omitted (deafult is false). If set to true it will spawn the robot in an unknown position (you can modify it in the nav_store.launch file). If you set it to be lost, you have to follow the instructions for the navigation step [here](http://wiki.ros.org/Robots/PMB-2/Tutorials/Navigation/Localization). Otherwise you can simply send a goal to the robot and it will find a path to it.

If you want to load a specific map of a specific store you have to provide them as command line parameters as e.g.
```bash
roslaunch storeplanner nav_store.launch task_num:=2 store_name:=store_2 traj_num:=1 map_name:=blender_map
```
Alternatively you can modify the parameters directly in the launch file by setting the desired default ones.

Inside the above launch file it is possible to enable the script (```wpoints_generator.py```) which is a ROS node performing the assigned task, as detailed in the paper.

### Navigation (Task 2)
To perform **Task 2** the only difference is the requirement to generate also the crossroad file. The steps are exactly as for **Task 1**.

#### Output
The metadata, made by the captured images together with info on the robot trajectory are stored in ```/acquisitions/<store_name>/<num_traj>/``` for each waypoint of the trajectory selected at the beginning.

### Additional notes
##### 1
The package is structured as follow:
- The store map created with the Mapping step should follow the convention to be placed in ```../maps/<store_name>/<map_name>```
- The same goes for trajectory files, that should be placed in ```../trajectories/<store_name>/``` as ```.json``` and numbered. As example see ```the store_2``` folders
##### 2
Pal Robotics uses a custom Rviz plugin for displaying covariance ellipses that is out-of-date. To avoid errors on the simulations (this however does not impact the simulation itself, just visualization) you should install an additional package in the workspace and build it
```bash
cd ~/<your_workspace>/src/pmb2_public_ws/src/
git clone https://github.com/pal-robotics/rviz_plugin_covariance.git
cd ~/<your_workspace>
catkin build
```

