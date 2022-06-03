# Master-thesis
Code used in my Master thesis 

All the credits are given in the corresponding folder. 

It is assumed that ROS noetic is already correctly working. 

To correctly work the following package are requiered :
- Cpu monitor : https://github.com/alspitz/cpu_monitor/blob/master/launch/cpu_monitor.launch
- Turtlebot3 description : https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_description
- Aws robotic environment : https://github.com/aws-robotics/aws-robomaker-bookstore-world
-  M-explore : https://github.com/hrnr/m-explore

Then, to perform an experiment on simulation :
-In the map_metrics folder, go to auto.py. Go to line 144 and change the path corresponding to your path to the file map_saver.launch. 
- If running an experiment with only one robot : change line 133 in auto.py from "/map_merge" to "tb3_0/map" and the same in the file "map_saver.launch" in the launch file. 
-choose one of the package amongst :Single_gmapping, Single_hector, multi_robot_exploration_2_hector, multi_robot_exploration_gmapping
- On the terminal run " roslaunch multi_robot_exploration two_tb_exploration.launch "
