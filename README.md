Installation:

```console	
$ cd ~/catkin_ws/src/auto_husky
$ chmod +x install_dependencies.sh 
$ ./install_dependencies.sh
```

Implementation:

1. Turn on ROS:
```console	
$ roscore
```
2. View published outputs that Husky will use to avoid closest obstacles:
```console	
$ rostopic echo /object_parameters
```
Note: The user has the ability to tune the filtering parameters and select how many objects to track

Note: Velocity is still under construction

3. Run particle_filtering simulation:
```console	
$ roslaunch auto_husky particle_sim.launch
```

4. Run particle_filtering with live LiDAR data:
```console	
$ roslaunch auto_husky particle_live.launch
```

Note: Greyscale particles are raw data. Rainbow particles are filtered particles. Filtetred particles are clustered together. White squares denote the center of each detected object cluster. Big green squares denote the nearest clusters (depending on number of prioritized objects selected).
