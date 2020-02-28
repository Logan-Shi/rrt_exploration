#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped 
import tf
from numpy import array,vstack,delete
from functions import gridValue,informationGain,isNew,isExplored,robot,dist
from sklearn.cluster import MeanShift
from rrt_exploration.msg import PointArray

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
new_frontiers = []
globalmaps=[]
localmaps=[]
def callBack(data,args):
	# print("msg received")
	global new_frontiers,min_distance
	new_frontiers = []
	for point in data.points:
		temp = PointStamped()
		temp.header.frame_id = mapData.header.frame_id
		temp.header.stamp = rospy.Time(0)
		temp.point.x = point.x
		temp.point.y = point.y
		temp.point.z = point.z
		transformedPoint=args[0].transformPoint(args[1],temp)
		x=[array([transformedPoint.point.x,transformedPoint.point.y])]
		if len(new_frontiers)>0:
			new_frontiers=vstack((new_frontiers,x))
		else:
			new_frontiers=x
    
def mapCallBack(data):
    global mapData
    mapData=data

def globalMap(data,indx):
	global globalmaps,namespace_init_count,n_robots
	if n_robots>1:
		_indx=indx+namespace_init_count
		#rospy.loginfo(str(_indx)+" globalmaps received!!!!!!!!!!!")
	elif n_robots==1:
		_indx=0
	globalmaps[_indx]=data

def localMap(data,indx):
	global localmaps,namespace_init_count,n_robots
	if n_robots>1:
		_indx=indx+namespace_init_count
		#rospy.loginfo(str(_indx)+" localmaps received!!!!!!!!!!!")
	elif n_robots==1:
		_indx=0
	localmaps[_indx]=data

# Node----------------------------------------------
def node():
	global frontiers,new_frontiers,mapData,globalmaps,localmaps,n_robots,namespace_init_count
	rospy.init_node('filter', anonymous=False)
	Robot = robot("/robot_1")
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	threshold= rospy.get_param('~costmap_clearing_threshold',70)
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	goals_topic= rospy.get_param('~goals_topic','/detected_points')	
	n_robots = rospy.get_param('~n_robots',1)
	namespace = rospy.get_param('~namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count',1)
	rateHz = rospy.get_param('~rate',100)
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	
#---------------------------------------------------------------------------------------------------------------
	
	for i in range(0,n_robots):
 		globalmaps.append(OccupancyGrid()) 
 		localmaps.append(OccupancyGrid()) 
 	
 	if len(namespace) > 0:	 
		rospy.Subscriber('robot_0/move_base_node/global_costmap/costmap', OccupancyGrid, globalMap,0)
		rospy.Subscriber('robot_1/move_base_node/global_costmap/costmap', OccupancyGrid, globalMap,1)
		rospy.Subscriber('robot_2/move_base_node/global_costmap/costmap', OccupancyGrid, globalMap,2)
		rospy.Subscriber('robot_0/map', OccupancyGrid, localMap,0)
		rospy.Subscriber('robot_1/map', OccupancyGrid, localMap,1)
		rospy.Subscriber('robot_2/map', OccupancyGrid, localMap,2)
	elif len(namespace)==0:
		rospy.Subscriber('/move_base_node/global_costmap/costmap', OccupancyGrid, globalMap,1) 	

#wait if map is not received yet
	while (len(mapData.data)<1):
		pass
#wait if any of robots' global costmap map is not received yet
	for i in range(0,n_robots):
 		 while (len(globalmaps[i].data)<1):
 		 	#rospy.loginfo("robot "+str(i)+"\'s globalmaps not received yet")
 		 	pass
	
	for i in range(0,n_robots):
 		 while (len(localmaps[i].data)<1):
 		 	#rospy.loginfo("robot "+str(i)+"\'s localmaps not received yet")
 		 	pass

	global_frame="/"+mapData.header.frame_id

	tfLisn=tf.TransformListener()
	rospy.loginfo("namespace_init_count: "+str(namespace_init_count))
	if len(namespace) > 0:
		for i in range(0,n_robots):
			tfLisn.waitForTransform(global_frame[1:], namespace+str(i+namespace_init_count)+'/base_link', rospy.Time(0),rospy.Duration(10.0))
	elif len(namespace)==0:
			tfLisn.waitForTransform(global_frame[1:], '/base_link', rospy.Time(0),rospy.Duration(10.0))
	
	rospy.Subscriber(goals_topic, PointArray, callback=callBack,callback_args=[tfLisn,global_frame[1:]])
	pub = rospy.Publisher('frontiers', Marker, queue_size=10)
	filterpub = rospy.Publisher('filtered_points', PointArray, queue_size=10)

	rospy.loginfo("the map and global costmaps are received")
	
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

	points.color.r = 0.0/255.0
	points.color.g = 0.0/255.0
	points.color.b = 255.0/255.0

	points.color.a=1;
	points.lifetime = rospy.Duration(1.0);

	p=Point()

	p.z = 0;
		
	temppoint=PointStamped()
	temppoint.header.frame_id= mapData.header.frame_id
	temppoint.header.stamp=rospy.Time(0)
	temppoint.point.z=0.0
	
	arraypoints=PointArray()
	tempPoint=Point()
	tempPoint.z=0.0
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
#-------------------------------------------------------------------------
#Clustering frontier points
		# centroids=[]
		# front=copy(frontiers)
		# if len(front)>1:
		# 	ms = MeanShift(bandwidth=0.3)   
		# 	ms.fit(front)
		# 	centroids= ms.cluster_centers_	 #centroids array is the centers of each cluster		

		# #if there is only one frontier no need for clustering, i.e. centroids=frontiers
		# if len(front)==1:
		# 	centroids=front
		# frontiers=copy(centroids)
		# wait if no frontier is received yet
		# print("new_frontiers: "+str(len(new_frontiers)))
		while len(new_frontiers)<1:
			pass
#-------------------------------------------------------------------------	
#clearing old frontiers 
		# print("started")
		filtered_frontiers = new_frontiers
		
		# print("filtered_frontiers: "+str(filtered_frontiers))

		for point in filtered_frontiers:
			temppoint.point.x=point[0]
			temppoint.point.y=point[1]
			for i in range(0,n_robots):
				transformedPoint=tfLisn.transformPoint(globalmaps[i].header.frame_id,temppoint)
				x=[transformedPoint.point.x,transformedPoint.point.y]
				# print("current list: "+str(frontiers))
				# print("current point: "+str(x))
				if (isNew(frontiers,x)):
					frontiers.append(copy(x))
		
		x,y = Robot.getPosition()
		for point in frontiers:
			cond=False
			for i in range(0,n_robots):
				cond=(gridValue(globalmaps[i],point)>threshold) or cond
				if cond or (dist([x,y],point)<5):
					frontiers.remove(point)

		# print("current frontiers: "+str(len(frontiers)))
#-------------------------------------------------------------------------
#publishing
		arraypoints.points=[]
		for i in frontiers:
			tempPoint.x=i[0]
			tempPoint.y=i[1]
			arraypoints.points.append(copy(tempPoint))
		filterpub.publish(arraypoints)

		pp=[]	
		for q in range(0,len(frontiers)):
			p.x=frontiers[q][0]
			p.y=frontiers[q][1]
			pp.append(copy(p))
		points.points=pp
		pub.publish(points)
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
