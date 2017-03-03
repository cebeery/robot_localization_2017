This folder contains bag files of a robot navigating maps created in AC109 as well as bags of our filter in action. 

<h3>Files</h3>

<h4>Navigation Files</h4> 
(included in Paul's base repo) <br>
ac109_1.bag <br>
ac109_2.bag <br>
ac109_3.bag <br>
ac109_4.bag 

<h4>Filter-in-action Files</h4> 
(referred to as "filtered bags"): <br>
ac109_1_filtered.bag <br>
ac109_2_filtered.bag <br>
ac109_3_filtered.bag <br>
ac109_4_filtered.bag 

<h3>Hows</h3>

Our filtered bags, labelled as "ac109_[map number]_filtered.bags", were recorded while both our particle filter and the robot navigation bags were running. <br> <br> 

To run the navigation bag files:<br>
$ roscore<br>
[new tab]<br>
$ rosparam set /use_sim_time true<br>
$ roslaunch neato_node set_urdf.launch<br>
$ roslaunch my_localizer test.launch map_file:=[path to repo]/robot_localization_2017/my_localizer/maps/ac109_[map number].yaml<br>
[new tab]<br>
$ rosrun rviz rviz<br>
[add map, map_pose, pose_cloud to rviz; set orentation to map]<br>
[new tab]<br>
$cd [path to bag folder]<br>
$rosbag play --clock bags/ac109_[map number].bag<br>
[set 2D pose arrow in rviz at pose of large red map pose arrow]<br><br>

To take filtered bag files:<br>
$ roscore<br>
[new tab]<br>
$ rosparam set /use_sim_time true<br>
$ roslaunch neato_node set_urdf.launch<br>
$ roslaunch my_localizer test.launch map_file:=[path to repo]/robot_localization_2017/my_localizer/maps/ac109_[map number].yaml<br>
[new tab]<br>
$ rosrun rviz rviz<br>
[add map, map_pose, pose_cloud to rviz; set orentation to map]<br>
[new tab]<br>
$ cd [path into bag folder]<br>
$ rosbag record -a -O [bag-file-name]<br>
[new tab]<br>
$rosbag play --clock bags/ac109_[map number].bag<br>
[set 2D pose arrow in rviz at pose of large red map pose arrow]<br>
[ctrl+C rosbag record once rosbag play ends<br><br>

To run filtered bag files:<br>
$ roscore<br>
[new tab]<br>
$ rosparam set /use_sim_time true<br>
$ roslaunch neato_node set_urdf.launch<br>
$ rosrun rviz rviz<br>
[add map, map_pose, pose_cloud to rviz; set orentation to map]<br>
[new tab]<br>
$cd [path to bag folder]<br>
$rosbag play --clock bags/ac109_[map number]_filtered.bag


<h3>Analysis</h3>

In general we see that the robot position once initialized is completely incorrect despite the particle cloud being located about the correct pose, but then corrects its location within 1-3 cycles. The particle filter stays spread out to about 2x the robot icon size through out. We also note that the filter has particular correctly locating itself during turns and corners.  The robot locating remains smooth after the first 1-3 cycles and stays approximatley in the correct location with regards to the real robot as seen by comparing the robot icon and map_pose arrow (red) as well as comparing the map walls with the laser scan. 

Some tuning on the filter attributes could be used to improve the turns and tighter convergence of the filter. 
