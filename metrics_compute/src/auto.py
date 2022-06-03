#!/usr/bin/env python3
#This code is used to automatically perform a benchmarking. Stop criterion is when 99% of the area is explored 
# Part of the code is from :https://github.com/efc-robot/Explore-Bench; related to the paper  "Explore-Bench: Data Sets, Metrics and Evaluations for Frontier-based and Deep-reinforcement-learning-based Autonomous Exploration" 



import sys
import time
import roslaunch
import os
import numpy as np
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped
import tf
import pickle
import yaml
from PIL import Image
import math
from numpy import float32, matrix, uint64
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from std_msgs.msg import Float32
 
start_time = time.time()
distance_robot_1 = 0
distance_robot_2 = 0
achieve_90 = False
odom_1=[] 
odom_2=[]
odom1=True
odom2= True
end_flag = False
exploration_rate_log = []
map_saved = False 
T_90 =0
gt_area = rospy.get_param("/total_ground_truth_area",167.095)
cpu = []
 
def callback(data): 
    # function partially from https://github.com/efc-robot/Explore-Bench;
    # This function compute the time to explore 90% and 99% of the map. When 99% of the map is explore the exploration is stopped. 
    # -1:unkown 0:free 100:obstacle
    global end_flag, start_time, achieve_90, T_90,distance_robot_2,distance_robot_1,cpu
    if not end_flag:
        gridmap = np.array(data.data).reshape((data.info.height, data.info.width))
        explored_map = (gridmap != -1).astype(int)
        explored_area = explored_map.sum()*data.info.resolution*data.info.resolution
        exploration_rate = explored_area / gt_area
        print("Exploration: ",explored_area)

        if exploration_rate >= 0.9 and (not achieve_90):
            print("achieve 0.9 coverage rate!")
            print("T_90: ", time.time()-start_time)
            T_90 = time.time()-start_time
            achieve_90 = True
        if exploration_rate >= 0.99:
            print("exploration ends!")
            print("T_90: ", T_90)
            print("T_total: ", time.time()-start_time)
            T_99 = time.time()-start_time         
            data = matrix([T_90, T_99,distance_robot_2+distance_robot_1,sum(cpu)/len(cpu)])
            header = "T_90, T_99, total_distance,average_CPU"
            f = open("/home/marin/catkin_ws/src/metrics_compute/Benchmarking_process.txt", 'w')
            save = np.savetxt(f, data,delimiter=',', header=header,fmt='%5.4g')
            end_flag = True        
        time.sleep(1)
    else:
        time.sleep(1)


def odom_callback_for_robot1(data):
        # function inspired from https://github.com/efc-robot/Explore-Bench;  
        # function computing the distance traveled by one robot using the odom data message. 
    global end_flag, distance_robot_1, odom1
    if not end_flag:
        current_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
        if len(odom_1) == 0:
            odom_1.append(current_pos)
            
        else:
            distance_traveled_1 = math.hypot(odom_1[-1][0]-current_pos[0], odom_1[-1][1]-current_pos[1])
            distance_robot_1 += distance_traveled_1
            odom_1.append(current_pos)
        time.sleep(1)
    else:
        if odom1 :
            print("distance travelled by robot_1 :",distance_robot_1)
            odom1 = False
        else :
            time.sleep(1)

def odom_callback_for_robot2(data):
        # function inspired from https://github.com/efc-robot/Explore-Bench;  
        # function computing the distance traveled by one robot using the odom data message. 
    global end_flag,distance_robot_1,distance_robot_2, odom2
    
    if not end_flag:
        current_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
        if len(odom_2) == 0:
            odom_2.append(current_pos)
            
        else:
            distance_traveled_2 = math.hypot(odom_2[-1][0]-current_pos[0], odom_2[-1][1]-current_pos[1])
            distance_robot_2 += distance_traveled_2
            odom_2.append(current_pos)
        time.sleep(1)
    else:
        if odom2 :
            print("distance travelled by robot_2 :",distance_robot_2)
            print("total distance travelled :",distance_robot_2+distance_robot_1)
            odom2 = False
        else :
            time.sleep(1)
            print("total distance travelled :",distance_robot_2+distance_robot_1)


def cpu_callback(data):
    global cpu
    cpu.append(data.data)




def main(argv):
    global gt_area, map_saved, end_flag
    
    rospy.init_node('exploration_metric', anonymous=True)
    rospy.Subscriber("/map_merge", OccupancyGrid, callback, queue_size=1)
    rospy.Subscriber("/tb3_0/odom", Odometry, odom_callback_for_robot1, queue_size=1)
    rospy.Subscriber("/tb3_1/odom", Odometry, odom_callback_for_robot2, queue_size=1)
    rospy.Subscriber("/cpu_monitor/total_cpu",Float32,cpu_callback)
    print(gt_area)

    while not rospy.is_shutdown():
        if end_flag and (not map_saved ) :
            map_saved = True 
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/marin/catkin_ws/src/metrics_compute/launch/map_saver.launch"])
            launch.start()
            rospy.loginfo("started")
            #Launch the python script to compute the Knn error and the Structure Similarity Index. 
            os.system("python3 /home/marin/catkin_ws/src/metrics_compute/src/Map_metrics.py /home/marin/catkin_ws/src/metrics_compute/map.pgm /home/marin/catkin_ws/src/metrics_compute/map_merge.pgm")  


if __name__ == '__main__':
    main(sys.argv)
