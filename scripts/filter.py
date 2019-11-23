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
from functions import gridValue,informationGain
from sklearn.cluster import MeanShift
from rrt_exploration.msg import PointArray

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
globalmaps=[]
localmaps=[]
def callBack(data,args):
	global frontiers,min_distance
	transformedPoint=args[0].transformPoint(args[1],data)
	x=[array([transformedPoint.point.x,transformedPoint.point.y])]
	if len(frontiers)>0:
		frontiers=vstack((frontiers,x))
	else:
		frontiers=x
    
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
	global frontiers,mapData,globalmaps,localmaps,n_robots,namespace_init_count
	rospy.init_node('filter', anonymous=False)
	
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
	
	rospy.Subscriber(goals_topic, PointStamped, callback=callBack,callback_args=[tfLisn,global_frame[1:]])
	pub = rospy.Publisher('frontiers', Marker, queue_size=10)
	pub2 = rospy.Publisher('centroids', Marker, queue_size=10)
	filterpub = rospy.Publisher('filtered_points', PointArray, queue_size=10)

	rospy.loginfo("the map and global costmaps are received")
	
	
	# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	
	
	points=Marker()
	points_clust=Marker()
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
	points.lifetime = rospy.Duration();

	p=Point()

	p.z = 0;

	pp=[]
	pl=[]
	
	points_clust.header.frame_id= mapData.header.frame_id
	points_clust.header.stamp= rospy.Time.now()

	points_clust.ns= "markers3"
	points_clust.id = 4

	points_clust.type = Marker.POINTS

#Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	points_clust.action = Marker.ADD;

	points_clust.pose.orientation.w = 1.0;

	points_clust.scale.x=0.2
	points_clust.scale.y=0.2 
	points_clust.color.r = 0.0/255.0
	points_clust.color.g = 255.0/255.0
	points_clust.color.b = 0.0/255.0

	points_clust.color.a=1;
	points_clust.lifetime = rospy.Duration();

		
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
		centroids=[]
		front=copy(frontiers)
		if len(front)>1:
			ms = MeanShift(bandwidth=0.3)   
			ms.fit(front)
			centroids= ms.cluster_centers_	 #centroids array is the centers of each cluster		

		#if there is only one frontier no need for clustering, i.e. centroids=frontiers
		if len(front)==1:
			centroids=front
		frontiers=copy(centroids)
#-------------------------------------------------------------------------	
#clearing old frontiers  
		z=0
		while z<len(centroids):
			cond=False
			temppoint.point.x=centroids[z][0]
			temppoint.point.y=centroids[z][1]
			for i in range(0,n_robots):
				transformedPoint=tfLisn.transformPoint(globalmaps[i].header.frame_id,temppoint)
				transformedPoint1=tfLisn.transformPoint(localmaps[i].header.frame_id,temppoint)
				x=array([transformedPoint.point.x,transformedPoint.point.y])
				x1=array([transformedPoint1.point.x,transformedPoint1.point.y])
				cond=(gridValue(globalmaps[i],x)>threshold or gridValue(localmaps[i],x1)>=100) or cond
			if (cond or (informationGain(mapData,[centroids[z][0],centroids[z][1]],info_radius*0.5))<0.2):
				centroids=delete(centroids, (z), axis=0)
				z=z-1
			z+=1
#-------------------------------------------------------------------------
#publishing
		arraypoints.points=[]
		for i in centroids:
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
		pp=[]	
		for q in range(0,len(centroids)):
			p.x=centroids[q][0]
			p.y=centroids[q][1]
			pp.append(copy(p))
		points_clust.points=pp
		pub.publish(points)
		pub2.publish(points_clust) 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
