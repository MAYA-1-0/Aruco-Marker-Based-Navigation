#!/usr/bin/env/python3
import cv2
import numpy as np
import cv2.aruco as aruco
import pyrealsense2 as rs
import math
centroid=(320,240)

class DepthCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def release(self):
        self.pipeline.stop()

dc=DepthCamera()

while True:
	ret, depth_frame, color_frame = dc.get_frame()
	gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
	#print(gray)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	arucoParameters = aruco.DetectorParameters_create()
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)
	frame = aruco.drawDetectedMarkers(color_frame, corners)
	#bbox=list(corners)
	
	if np.all( ids != None):
		b1=(corners[0][0][0][0],corners[0][0][0][1])
		b2=(corners[0][0][1][0],corners[0][0][1][1])
		b3=(corners[0][0][2][0],corners[0][0][2][1])
		b4=(corners[0][0][3][0],corners[0][0][3][1])
		x=((b2[0]-b1[0])+(b3[0]-b4[0]))/2
		y=((b3[1]-b2[1])+(b4[1]-b1[1]))/2
		#C=( int(((b2[0]-b1[0])+(b3[0]-b4[0]))/2),int( ((b4[1]-b1[1])+(b3[1]-b2[1]))/2) ) 
		C=(int(b1[0]+x/2),int(b1[1]+y/2))
		C1=(int(b1[0]+x/2),int(centroid[1]))
		#print('Centroid of bounding box',C)
		#print("bounding boxes",b1,b2,b3,b4)
		#print(corners)
		cv2.circle(color_frame, C, 3, (0, 0, 255))
		d_m = depth_frame[int(C[1]),int(C[0])]
		d_m1=depth_frame[int(C1[1]),int(C1[0])]
		#d=depth_frame[int(centroid[1]),int(centroid[0])]
		print('Markers Depth',d_m1)
		if C[0]<centroid[0]:
			offset=-1
		else:
			offset=1
		#a=d/d_m1
		#b=math.sqrt(d_m1**2-d**2)
		d=depth_frame[int(centroid[1]),int(centroid[0])]
		a=d/d_m1
		print(a)
		#theta=offset*math.acos(a)
		#print("d and theta",d,(theta*180)/math.pi)
		cv2.putText(color_frame, "Marker detected",(10,20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
	else:
		cv2.putText(color_frame, "No Marker ids Detected", (10,20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
	cv2.imshow("Detection frame", frame)
	d=depth_frame[int(centroid[1]),int(centroid[0])]
	print("centroid depth",d)
	#cv2.imshow("depth frame", depth_frame)
	# print(color_frame)
	key = cv2.waitKey(1)
	if key == 27:
		dc.release()
		break






