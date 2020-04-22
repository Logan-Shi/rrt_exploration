#!/usr/bin/env python


#--------Include modules---------------
from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid
from functions import robot
from sklearn.cluster import MeanShift

import numpy as np
import cv2
import math

#-----------------------------------------------------

def getfrontier(mapData):
	debug = 0
	# laserRange = 10
	# robots=[]
	# robots.append(robot("/robot_1"))
	# x,y = robots[0].getPosition()
	data=mapData.data
	w=mapData.info.width
	h=mapData.info.height
	resolution=mapData.info.resolution
	# r = int(laserRange / resolution)+1
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y
	#rospy.loginfo("x: "+str(Xstartx)+" y: "+str(Xstarty))
	# print("robot at : "+str(x)+str(y))
	# print("map : "+str(w)+str(h))
	
	# xRobot = int((x - Xstartx)/resolution)
	# yRobot = int((y - Xstarty)/resolution)
	# print("Robot index in map : "+str(xRobot)+str(yRobot))
	# print("x1,y1: "+str([xRobot-r,yRobot-r])+"x2,y2: "+str([xRobot+r,yRobot+r]))
	rimg = np.zeros((h, w, 1), np.uint8)
	threshold = 50
	
	for i in range(0,h):
		for j in range(0,w):
			if data[i*w+j]>threshold:# walls
				rimg[i,j]=0
			elif data[i*w+j]==-1:# unexplored
				rimg[i,j]=205
			else: # free space
				rimg[i,j]=255
	# x1 = xRobot-r
	# if x1 < 0:
	# 	x1 = 0
	# y1 = yRobot-r
	# if y1 < 0:
	# 	y1 = 0
	# x2 = xRobot+r
	# if x2 > w:
	# 	x2 = w
	# y2 = yRobot+r
	# if y2 > h:
	# 	y2 = h
	img = rimg

	o=cv2.inRange(img,0,1)

	# kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
	# img1=cv2.dilate(img,kernel)
	#cv2.imshow('img1',img1)
	edges = cv2.Canny(img,0,255)
	#cv2.imshow('edges',edges)
	
	im2, contours, hierarchy = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(o, contours, -1, (255,255,255), 50)

	o=cv2.bitwise_not(o) 

	# kernel1= cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
	# o1=cv2.erode(o,kernel1)
	# cv2.imshow('o1',o1)

	res = cv2.bitwise_and(o,edges)
	# rospy.loginfo("t1")
	res1=cv2.resize(res,(0,0),fx=0.2,fy=0.2,interpolation=cv2.INTER_NEAREST)
	# rospy.loginfo("t2")
	#cv2.imshow('res1',res1)
	#------------------------------
       #frontier=copy(res)
	frontier=copy(res1)
	im2, contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(frontier, contours, -1, (255,255,255), 2)

	im2, contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	all_pts=[]

	if debug:
		cv2.imshow('img',img)
		cv2.imshow('edges',edges)
		cv2.imshow('o',o)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

	if len(contours)>0:
		upto=len(contours)-1
		i=0
		maxx=0
		maxind=0
		
		for i in range(0,len(contours)):
				cnt = contours[i]
				M = cv2.moments(cnt)
				#cx = int(M['m10']/M['m00'])
				cx = 5*int(M['m10']/M['m00'])
				#cy = int(M['m01']/M['m00'])
				cy = 5*int(M['m01']/M['m00'])
				xr=cx*resolution+Xstartx
				yr=cy*resolution+Xstarty
				pt=[np.array([xr,yr])]
				if len(all_pts)>0:
					all_pts=np.vstack([all_pts,pt])
				else:
							
					all_pts=pt
	
	return all_pts

def croatiangetfrontier(mapData):
	debug = 0

	data=mapData.data
	w=mapData.info.width
	h=mapData.info.height
	resolution=mapData.info.resolution

	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y

	threshold = 50
	rimg = np.zeros((h, w, 1), np.uint8)
	frontiers = []
	for i in range(0,h):
		for j in range(0,w):
			if data[i*w+j]==-1 and isFrontier(i,j,mapData):
				frontiers.append([j*resolution+Xstartx,i*resolution+Xstarty])

	if debug:
		cv2.imshow('rimg',rimg)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

	centroids=[]
	front=copy(frontiers)
	if len(front)>1:
		ms = MeanShift(bandwidth=1)   
		ms.fit(front)
		centroids= ms.cluster_centers_	 #centroids array is the centers of each cluster		
	#if there is only one frontier no need for clustering, i.e. centroids=frontiers
	if len(front)==1:
		centroids=front
	frontiers=copy(centroids)
	
	return frontiers

def isFrontier(x,y,mapData):
	data=mapData.data
	w=mapData.info.width
	h=mapData.info.height
	resolution=mapData.info.resolution
	isFront = False
	threshold = 50
	if isFree(x,y+1,mapData) or isFree(x,y-1,mapData):
		isFront = True

	return isFront

def isFree(i,j,mapData):
	data=mapData.data
	w=mapData.info.width
	h=mapData.info.height
	resolution=mapData.info.resolution
	threshold = 50

	# print("rows: "+str(h)+" cols: "+str(w))
	# print("row: "+str(i)+" col: "+str(j))
	if i>=h or j>=w or j<0 or i<0:
		# print("invalid return")
		return False

	if data[i*w+j]>=0 and data[i*w+j]<threshold:
		return True
	return False

def recData(mapData):
	debug = 0
	# laserRange = 10
	# robots=[]
	# robots.append(robot("/robot_1"))
	# x,y = robots[0].getPosition()
	data=mapData.data
	w=mapData.info.width
	h=mapData.info.height
	resolution=mapData.info.resolution
	# r = int(laserRange / resolution)+1
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y
	#rospy.loginfo("x: "+str(Xstartx)+" y: "+str(Xstarty))
	# print("robot at : "+str(x)+str(y))
	# print("map : "+str(w)+str(h))
	
	# xRobot = int((x - Xstartx)/resolution)
	# yRobot = int((y - Xstarty)/resolution)
	# print("Robot index in map : "+str(xRobot)+str(yRobot))
	# print("x1,y1: "+str([xRobot-r,yRobot-r])+"x2,y2: "+str([xRobot+r,yRobot+r]))
	rimg = np.zeros((h, w, 1), np.uint8)
	threshold = 50
	
	entropy = 0
	prob = np.zeros((h,w,1),np.uint32)
	frec = open("test.txt","a")
	for i in range(0,h):
		for j in range(0,w):
			if data[i*w+j]>threshold:# walls
				# prob[i,j] = 1
				entropy = entropy + 0.5 * math.log(0.5)
				rimg[i,j]=0
			elif data[i*w+j]==-1:# unexplored
				rimg[i,j]=205
			else: # free space
				# prob[i,j] = 0
				entropy = entropy + 0.5 * math.log(0.5)
				rimg[i,j]=255
	time = rospy.get_rostime()
	t = time.secs
	frec.write(str(t)+';'+str(entropy)+'\n')
	frec.close()