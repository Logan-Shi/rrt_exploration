#!/usr/bin/env python


#--------Include modules---------------
from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2

#-----------------------------------------------------

def getfrontier(mapData):
	data=mapData.data
	w=mapData.info.width
	h=mapData.info.height
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y
	#rospy.loginfo("x: "+str(Xstartx)+" y: "+str(Xstarty))
	 
	img = np.zeros((h, w, 1), np.uint8)
	
	for i in range(0,h):
		for j in range(0,w):
			if data[i*w+j]==100:# walls
				img[i,j]=0
			elif data[i*w+j]==-1:# unexplored
				img[i,j]=205
			else:
				img[i,j]=255

	o=cv2.inRange(img,0,1)

	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
	img1=cv2.dilate(img,kernel)
	edges = cv2.Canny(img1,0,255)
	
	im2, contours, hierarchy = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(o, contours, -1, (255,255,255), 5)

	o=cv2.bitwise_not(o) 
	kernel1= cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
	o1=cv2.erode(o,kernel1)

	res = cv2.bitwise_and(o1,edges)
	res1=cv2.resize(res,(0,0),fx=0.2,fy=0.2,interpolation=cv2.INTER_NEAREST)
	#------------------------------
	frontier=copy(res1)
	im2, contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(frontier, contours, -1, (255,255,255), 2)

	im2, contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	all_pts=[]
	

	if len(contours)>0:
		upto=len(contours)-1
		i=0
		maxx=0
		maxind=0
		
		for i in range(0,len(contours)):
				cnt = contours[i]
				M = cv2.moments(cnt)
				cx = 5*int(M['m10']/M['m00'])
				cy = 5*int(M['m01']/M['m00'])
				xr=cx*resolution+Xstartx
				yr=cy*resolution+Xstarty
				pt=[np.array([xr,yr])]
				if len(all_pts)>0:
					all_pts=np.vstack([all_pts,pt])
				else:
							
					all_pts=pt
	
	return all_pts

