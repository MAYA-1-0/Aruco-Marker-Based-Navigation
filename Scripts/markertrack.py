#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import cv2.aruco as aruco
import pyrealsense2 as rs
import math
from nav.msg import Pose
from std_msgs.msg import Float64MultiArray
import signal
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
pub_dt =rospy.Publisher("d_t",Float64MultiArray,queue_size=1)
pub=rospy.Publisher("maya/marker_pose",Pose,queue_size=10)
#goal_pub=rospy.Publisher("/base/goal",Twist,queue_size=10)
rospy.init_node("marker_node",anonymous=True)
rate=rospy.Rate(10)
centroid=(240,320)
distance1=0
theta1=0
dt = [0,0]
def keyboardInterruptHandler(signal, frame):
	print("interrupt detected")
	dc.release()
	exit(0)


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
new_goal = True


while True:
	ret,depth_frame,color_frame=dc.get_frame()
	gray=cv2.cvtColor(color_frame,cv2.COLOR_BGR2GRAY)
	aruco_dict=aruco.Dictionary_get(aruco.DICT_5X5_250)
	arucoParameters=aruco.DetectorParameters_create()
	corners,ids,rejectedImgPoints=aruco.detectMarkers(gray,aruco_dict,parameters=arucoParameters)
	message=Pose()
	if np.all(ids !=None ):
		b = corners[0][0][3][1]-corners[0][0][0][1]
		a = corners[0][0][2][1]-corners[0][0][1][1]
		h = float(((corners[0][0][1][0]-corners[0][0][0][0])+(corners[0][0][2][0]-corners[0][0][3][0]))/2.0)
		center_y = int((corners[0][0][3][1]-corners[0][0][0][1])/2 + corners[0][0][0][1])
		delta_x = (((b+2.0*a)/(3.0*(a+b)))*h)
		center_x = int(delta_x+(corners[0][0][0][0]+corners[0][0][3][0])/2.0)
		if b > a:
			theta= -1.0
		else:
			theta= 1.0
		try:
			theta1=theta*math.acos(float(h)/float(b))
			theta1=math.degrees(theta1)
			distance1=float(depth_frame[center_y,center_x])/1000.0
			aligned1=(abs(center_x-320)<abs(h))
			id1=ids[0][0]
			font=cv2.FONT_HERSHEY_PLAIN
			fontscale=0.4
			thickness=1
			yellow=(255,255,0)
			id="Marker ID : {} ".format(id1)
			theta_="Theta : {:.2f} ".format(theta1)
			dist= "Distance : {:.2f} ".format(distance1)
			align="align : {} ".format(aligned1)
			print("distance : {} ,aligned : {} and theta : {}".format(distance1,aligned1,theta1))
			message.detected=True
			message.id=id1
			#message.distance=distance1
		#	message.theta=theta1
#			new_goal = Truenano src/nav/scripts/test/detect.py
			dt[0]= distance1
			dt[1] = theta1
			position = Float64MultiArray(data=dt)
			if new_goal:
				pub_dt.publish(position)
				new_goal =False
			else:
				pass
			message.aligned=aligned1
			pub.publish(message)
			rate.sleep()
			cv2.putText(color_frame,str(id)+str(theta_)+str(align),(20,40),font,1.5,(0,255,0),4)
		except ValueError:
			rospy.logwarn("Error Calculating theta with values {},{}".format(h,b))
			detected=False
			message.id=0
			message.distance=0
		#	message.theta=0
		#	message.aligned=0
			pub.publish(message)
		#	rate.sleep()
		except ZeroDivisionError:
			rospy.logwarn("Zero Division Error")
			message.id=0
			detected=False
		#	message.distance=0
		#	message.theta=0
			message.aligned=0
			pub.publish(message)
		#	rate.sleep()
		except IndexError as e:
			rospy.logwarn("Index out of bounds: {}".format(e))
			message.id=0
			detected=False
		#	message.distance=0
		#	message.theta=0
			message.aligned=0
			pub.publish(message)
		#	rate.sleep()
	else:
		print("No marker Detected")
		message.id=0
		message.detected=False
	#	message.distance=0
	#	message.theta=0
		message.aligned=0
		pub.publish(message)
		rate.sleep()

	frame=aruco.drawDetectedMarkers(color_frame,corners)
	#cv2.imshow("detection frame",frame)
	#cv_image = bridge.cv2_to_imgmsg(frame,"bgr8")
	#img_pub.Publish(cv_image)
	signal.signal(signal.SIGINT, keyboardInterruptHandler)
	if cv2.waitKey(1) == ord('q'):
		dc.release()
		break













