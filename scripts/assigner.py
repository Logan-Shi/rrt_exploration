#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from rrt_exploration.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount,isNew,dist
from numpy.linalg import norm

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
globalmaps=[]
def callBack(data):
	global frontiers
	for point in data.points:
		if isNew(frontiers,[point.x,point.y]):
			frontiers.append([point.x,point.y,-99.0,0.0,99.0])

def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------

def node():
	global frontiers,mapData,globalmaps
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
	n_robots = rospy.get_param('~n_robots',1)
	namespace = rospy.get_param('~namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count',1)
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
	rateHz = rospy.get_param('~rate',100)
	
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
#---------------------------------------------------------------------------------------------------------------
		
# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass

#wait if map is not received yet
	while (len(mapData.data)<1):
		pass

	robots=[]
	if len(namespace)>0:
		for i in range(0,n_robots):
			robots.append(robot(namespace+str(i+namespace_init_count)))
	elif len(namespace)==0:
			robots.append(robot(namespace))
	for i in range(0,n_robots):
		robots[i].sendGoal(robots[i].getPosition())
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		print("current frontiers: "+str(len(frontiers)))
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		for ip in range(0,len(frontiers)):
			if frontiers[ip][3] == 0.0:
				frontiers[ip][2] = informationGain(mapData,[frontiers[ip][0],frontiers[ip][1]],info_radius)
				rospy.loginfo("added infoGain"+str(frontiers[ip][2]))
				frontiers[ip][3] = 1.0
			else:
				print("already calced, skip")
		# print("current frontiers: "+str(frontiers))
#-------------------------------------------------------------------------			
#get number of available/busy robots
		# na=[] #available robots
		# nb=[] #busy robots
		# for i in range(0,n_robots):
		# 	if (robots[i].getState()==1):
		# 		nb.append(i)
		# 	else:
		# 		na.append(i)
		#if len(na)>0:
			#rospy.loginfo("available robots: "+str(na))	
		# if len(frontiers)>0:
		# 	rospy.loginfo("current number of frontiers: "+str(len(frontiers)))	
#------------------------------------------------------------------------- 
#get dicount and update informationGain
		# for i in nb+na:
		# 	infoGain=discount(mapData,robots[i].assigned_point,frontiers,infoGain,info_radius)
#-------------------------------------------------------------------------            
		revenue_record=[]
		centroid_record=[]
		id_record=[]
		
		# for ir in na:
		# 	for ip in range(0,len(frontiers)):
		# 		cost=norm(robots[ir].getPosition()-frontiers[ip])		
		# 		threshold=1
		# 		information_gain=infoGain[ip]
		# 		if (norm(robots[ir].getPosition()-frontiers[ip])<=hysteresis_radius):

		# 			information_gain*=hysteresis_gain
		# 		revenue=information_gain*info_multiplier-cost
		# 		revenue_record.append(revenue)
		# 		centroid_record.append(frontiers[ip])
		# 		id_record.append(ir)
		
		#if len(na)<1:
		info_record=[]
		revenue_record=[]
		centroid_record=[]
		id_record=[]
		ir = 0
		x,y = robots[ir].getPosition()
		for ip in frontiers:
			# print("cost: "+str(frontiers))
			cost = dist([x,y],[ip[0],ip[1]])
			ip[4] = cost
			print("cost calced"+str(ip))

		for ip in frontiers:
			if ip[4]<4:
				print("removed: "+str(ip))
				print(str(cost))
				frontiers.remove(ip)
		
		for ip in range(0,len(frontiers)):
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
		print("current frontiers list: "+str(frontiers))
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
			rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id])+" with revenue of "+str(revenue_record[winner_id]))	
			rospy.sleep(delay_after_assignement)
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
