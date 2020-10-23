#include <ros/ros.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
	// Begin node /////////////////////////////////////////////
	
	init(argc, argv, "bag_node");
	NodeHandle nh;

	string rosbag_cmd = "rosbag play /home/$USER/particle_filtering.bag"; 
	const char *command = rosbag_cmd.c_str(); 

	cout << "Compiling file using " << command << endl; 
	system(command); 
	return 0;
}

