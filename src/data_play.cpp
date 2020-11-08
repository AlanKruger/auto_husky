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

	// Enters this string into a terminal automatically

	string rosbag_cmd = "rosbag play /home/$USER/vel_charan.bag"; 
	const char *command = rosbag_cmd.c_str();

	// Give feedback

	cout << "Compiling file using " << command << endl; 
	system(command); 
	return 0;
}

