#!/usr/bin/env python


#--------Include modules---------------
import rospy
from copy import copy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from rrt_exploration.msg import PointArray
from getfrontier import getfrontier,croatiangetfrontier
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

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
		map_topic= rospy.get_param('~map_topic','/robot_1/submap')# can't be map/merged_map, because when initialized, it's not published yet
		rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
		targetspub = rospy.Publisher('/detected_points', PointArray, queue_size=10)
		rospy.init_node('detector', anonymous=False)
# wait until map is received, when a map is received, mapData.header.seq will not be < 1
		while mapData.header.seq<1 or len(mapData.data)<1:
			pass

		pub = rospy.Publisher('new_frontiers', Marker, queue_size=10)
		points=Marker()
#Set the frame ID and timestamp.  See the TF tutorials for information on these.
		points.header.frame_id= mapData.header.frame_id
		points.header.stamp= rospy.Time.now()
	
		points.ns= "markers2"
		points.id = 0
		
		points.type = Marker.POINTS
		
#Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		points.action = Marker.ADD;
	
		points.pose.orientation.w = 1.0
	
		points.scale.x=0.1
		points.scale.y=0.1 
	
		points.color.r = 255.0/255.0
		points.color.g = 0.0/255.0
		points.color.b = 0.0/255.0
	
		points.color.a=1;
		points.lifetime = rospy.Duration(1.0);
		rate = rospy.Rate(2)
#-------------------------------OpenCV frontier detection------------------------------------------
		while not rospy.is_shutdown():
			# print("running")
			frontiers=getfrontier(mapData)
			# print("points detected")
			arraypoints.points=[]
			for i in range(len(frontiers)):
				x=frontiers[i]
				exploration_goal.x=x[0]
				exploration_goal.y=x[1]
				exploration_goal.z=0	
				arraypoints.points.append(copy(exploration_goal))
			targetspub.publish(arraypoints)

			pp=[]	
			for q in arraypoints.points:
				pp.append(copy(q))
			points.points=pp
			pub.publish(points)
			rate.sleep()
#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass