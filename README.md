copy the packages into the src folder of your workspace.
B, V, and D subscripts stand for three different rosbots, but one can also just use one or two rosbots. V or no subscript corresponds with the master ROSbot
Lane detection can be launched via the launch file...
Overtaking maneuver with lane detection can be launched via the launch files V1, B1, and D1, where V1 has to be launched on the overtaking ROSbot. These launch files can be found in the datmo package (e.g. “roslaunch datmo V1.launch”).

Setup for two rosbots:

Both in right lane, one behind the other with the slow one in front.

Setup for three rosbots:

same as for two, except now there is a third one on the left lane, at some distance behind the overtaking ROSbot.

Packages:

The image_proc_lane grabs the feed from the camera and processes it. the output is an array with the coordinates of the final Hough lines. 

The draw_houghlines package draws lines using the coordinates from the image_proc_lane package and draws these on top of the camera feed image. 

The datmo package organises the lidar data into clusters.

The vehicle2 package filters the objects returned by the datmo packages to objects in a closer predefined range of the vehicle running the file.

The lidarcom package identifies other vehicles and access associated information. Then it performs calculations and makes a decision on what action to take. The actions are numbers, and a number associated with an action is published to a command topic, read by the switch package.

Cruise_B and Cruise_D packages are used on the non-overtaking rosbots. The packages set a speed for the ROSbots and read the Hough lines to stay in the lane. The default for Cruise_B is the fast vehicle on the left lane, while the default for Cruise_D is the slow vehicle on the right lane.

rot package Does the same for the overtaking ROSbot as the cruise packages does for the other ones.

Switch package is the final package on the overtaking ROSbot. This package reads the commands from lidarcom, the y and x position of itself and overwrites the rot package to change lanes.

Stop packages. for each ROSbot there is a seperate stop packages. After terminating the launch file the ROSbots will still drive. By running the stop packages after it, the ROSbots will stop moving and are safe to pick up.
