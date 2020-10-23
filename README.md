Installation:

Under Construction...

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
Note: Greyscale particles are raw data. Rainbow particles are filtered particles. Filtetred particles are clustered together. White squares denote the center of each detected object cluster. Big green squares denote the nearest clusters (depending on number of prioritized objects selected).

Note: Running with Velodyne drivers (using real sensor and not simulated data) is under construction
