#!/usr/bin/env python


#--------Include modules---------------
import rospy
from copy import copy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from rrt_exploration.msg import PointArray
from getfrontier import getfrontier
from geometry_msgs.msg import Point

#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()

def mapCallBack(data):
    global mapData
    mapData=data

# Node----------------------------------------------
def node():
		global mapData
		exploration_goal=Point()
		arraypoints=PointArray()
		map_topic= rospy.get_param('~map_topic','/robot_1/map')# can't be map/merged_map, because when initialized, it's not published yet
		rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
		targetspub = rospy.Publisher('/detected_points', PointArray, queue_size=10)
		rospy.init_node('detector', anonymous=False)
# wait until map is received, when a map is received, mapData.header.seq will not be < 1
		while mapData.header.seq<1 or len(mapData.data)<1:
			pass
    	   	
		rate = rospy.Rate(50)	
#-------------------------------OpenCV frontier detection------------------------------------------
		while not rospy.is_shutdown():
			frontiers=getfrontier(mapData)
			arraypoints.points=[]
			for i in range(len(frontiers)):
				x=frontiers[i]
				exploration_goal.x=x[0]
				exploration_goal.y=x[1]
				exploration_goal.z=0	
				arraypoints.points.append(copy(exploration_goal))
			targetspub.publish(arraypoints)
			rate.sleep()
#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass