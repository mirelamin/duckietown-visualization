# duckietown-visualization
This is a ROS package for visualization of Duckietown maps and duckiebots.

## Features
- [x] Visualization of maps from duckietown-world
- [ ] Visualization of road signs and watchtowers
- [ ] Realtime visualization of duckiebots

## Prerequisites
- Desktop-Full installation of ROS
- [duckietown-world](https://github.com/duckietown/duckietown-world)

## Installing
From the `src` directory of your ROS Workspace, run
```
$ git clone https://github.com/surirohit/duckietown-visualization
```
From your workspace directory, run
```
$ catkin build 
```
Run `catkin_make` instead if you don't use `python-catkin-tools`.

Next, source your workspace using
```
$ source devel/setup.zsh
```
Run `source devel/setup.bash` instead if you use bash.

## Running the map visualization
Run the visualization of the `robotarium1` map, which is currently the default 
by using
```
$ roslaunch duckietown_visualization publish_map.launch
```

You can specify different map names to be loaded according to the maps in 
`duckietown-world`. For example,
```
$ roslaunch duckietown_visualization publish_map.launch map_name:="small_loop"
```
You'll notice that in this case, the RViz visualization has the map loaded at 
the corner. This is because the default configuration for RViz (located in `duckietown_visualization/config/default.rviz`) is configured for the 
`robotarium1` map. You can load your own rviz configuration by running
```
$ roslaunch duckietown_visualization publish_map.launch map_name:="small_loop" rviz_config:="path/to/myconfig.rviz"
```


## How it works

To understand the working of this package, you need a basic understanding of the
[ROS Transform library](http://wiki.ros.org/tf2) and 
[RViz Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker). This package reads
the map having name `map_name` from `duckietown-world` and broadcasts markers
([MESH_RESOURCE](http://wiki.ros.org/rviz/DisplayTypes/Marker#Mesh_Resource_.28MESH_RESOURCE.3D10.29_.5B1.1.2B-.5D)) 
for each element of the map. Each class of elements (tiles, road signs, etc.) is 
published in a different namespace under the transform `/map` to provide the 
feature of turning off a certain category of map elements.

