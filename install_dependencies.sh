#!/bin/bash

echo "Installing dependencies..."

if [ -d "/home/$USER/catkin_ws/" ] 
then
	echo "catkin_ws exists..." 
	cd /home/$USER/catkin_ws/
	rosdep update
	rosdep install --from-paths src -i

	echo "All dependencies for auto_husky were installed successfully :)"

else
	echo "Error: Directory ~/catkin_ws does not exists."
	exit $1
fi
