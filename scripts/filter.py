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
from rrt_exploration.msg import PointArray

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
globalmaps=[]
localmaps=[]
callbackTime = rospy.Time()
haveNew = 0.0
def callBack(data,args):
	# print("msg received")
	global frontiers,globalmaps,callbackTime,info_radius
	temp = PointStamped()
	temp.header.frame_id = mapData.header.frame_id
	temp.header.stamp = rospy.Time(0)
	# print("called")
	for point in data.points:
		temp.point.x = point.x
		temp.point.y = point.y
		transformedPoint=args[0].transformPoint(globalmaps[0].header.frame_id,temp)
		
		x=[transformedPoint.point.x,transformedPoint.point.y]
		# print("current list: "+str(frontiers))
		# print("current point: "+str(x))
		needExplore = isExplored(mapData,x)
		flagNew = (isNew(frontiers,x,info_radius))
		# print("needExplore: "+str(needExplore))
		# print("flagNew: "+str(flagNew))
		if needExplore and flagNew:
			# print("append")
			frontiers.append([x[0],x[1],-99.0,0.0,99.0])
	# print("finish")

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
	global frontiers,mapData,globalmaps,localmaps,n_robots,namespace_init_count,callbackTime,buf,info_radius
	rospy.init_node('filter', anonymous=False)
	buf = 1001
	Robot = robot("/robot_1")
	Robot.sendGoal(Robot.getPosition())
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	threshold= rospy.get_param('~costmap_clearing_threshold',70)
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	goals_topic= rospy.get_param('~goals_topic','/detected_points')
	info_multiplier=rospy.get_param('~info_multiplier',3.0)
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
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

	points.scale.x=0.2
	points.scale.y=0.2 

	points.color.r = 0.0/255.0
	points.color.g = 0.0/255.0
	points.color.b = 255.0/255.0

	points.color.a=1;
	points.lifetime = rospy.Duration(1.0);

	p=Point()

	p.z = 0;
	
	arraypoints=PointArray()
	tempPoint=Point()
	tempPoint.z=0.0

	callbackTime = rospy.get_rostime()
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		# print("started")
		haveNew = 0.0
		buf+=1
		x,y = Robot.getPosition()
		if len(frontiers)>0:
			for ip in frontiers:
				# print("cost: "+str(frontiers))
				cost = dist([x,y],[ip[0],ip[1]])
				ip[4] = cost
				# print("cost calced"+str(ip))
			# print("finished cost calc")
			for ip in frontiers:
				cond=False
				for i in range(0,n_robots):
					cond=(gridValue(globalmaps[i],[ip[0],ip[1]])>10) or cond
					cond = (not isExplored(mapData,[ip[0],ip[1]])) or cond
					if cond:# or (ip[4] < info_radius):
						# print("removed: "+str(ip))
						# print(str(cost))
						frontiers.remove(ip)
		else:
			callbackTime = rospy.get_rostime()
		# print("finished remove")
		for ip in range(0,len(frontiers)):
			if frontiers[ip][3] == 0.0:
				frontiers[ip][2] = informationGain(mapData,[frontiers[ip][0],frontiers[ip][1]],info_radius)
				haveNew = 1.0
				# rospy.loginfo("added infoGain"+str(frontiers[ip][2]))
				frontiers[ip][3] = 1.0
			# else:
				# print("already calced, skip")
		
		now = rospy.get_rostime()
		delay = (now.secs - callbackTime.secs)
		# print("time stamp %i %i",now.secs,callbackTime.secs)
		# print("delay"+str(delay))
		# print("buf"+str(buf))
		if (buf > 500): # and (haveNew) or (delay>10):
			buf = 0
			# Robot.cancelGoal()
			info_record=[]
			revenue_record=[]
			centroid_record=[]
			id_record=[]
			ir = 0
			
			
			for ip in range(0,len(frontiers)):

				if frontiers[ip][4] < 0:
					continue

				if (delay>10):
					frontiers[ip][2] = informationGain(mapData,[frontiers[ip][0],frontiers[ip][1]],info_radius)

				information_gain=frontiers[ip][2]
				cost = frontiers[ip][4]
				# if (norm(robots[ir].getPosition()-frontiers[ip])<=hysteresis_radius):
				# 	information_gain*=hysteresis_gain
			
				# if ((norm(frontiers[ip]-robots[ir].assigned_point))<hysteresis_radius):
				# 	information_gain=informationGain(mapData,[frontiers[ip][0],frontiers[ip][1]],info_radius)*hysteresis_gain
				revenue=information_gain*info_multiplier-cost
				# rospy.loginfo("info record: "+str(information_gain*info_multiplier))	
				# rospy.loginfo("cost record: "+str(cost))
				# rospy.loginfo("revenue record: "+str(revenue))
				info_record.append(information_gain)
				revenue_record.append(revenue)
				centroid_record.append(frontiers[ip])
				id_record.append(ir)
		
	#-------------------------------------------------------------------------	
			# print("current frontiers list: "+str(frontiers))
			if (len(id_record)>0):
				callbackTime = rospy.get_rostime()
				winner_id=revenue_record.index(max(revenue_record))
				Robot.sendGoal(centroid_record[winner_id])
				rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id])+" with revenue of "+str(revenue_record[winner_id]))	
				# rospy.sleep(delay_after_assignement)
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
		
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
