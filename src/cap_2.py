#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image, Joy # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy
import math
from duckietown_msgs.msg import Twist2DStamped

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self.callback)
		rospy.Subscriber("/duckiebot/joy", Joy, self.callback_control)
		self.pub = rospy.Publisher("/duckiebot/camera_node/image/test", Image, queue_size=1)
		self.pub_control = rospy.Publisher("duckiebot/wheels_driver_node/car_cmd", Twist2DStamped)
		self.detector = cv2.CascadeClassifier("/home/duckiebot/duckietown/catkin_ws/src/ros_cap/src/cascade3(LBP).xml")

	def callback(self,msg):
		# Camara
		
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(msg, "bgr8")
		
		img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		
		dets = self.detector.detectMultiScale(img_gray, 1.3, 10)

		min_area = 5
		for val in dets:
			x, y, w, h = val
			if w*h > min_area:
				cv2.rectangle(image, (x+w, y+h), (x,y), (0, 0, 0), 2)
		
		msg = bridge.cv2_to_imgmsg(image, "bgr8")

		self.pub.publish(msg)
		
	def callback_control(self, msg):
		axes = list(msg.axes)
		buttons = list(msg.buttons)

		B = buttons[1]
		
		if B == 1:
			for i in range(len(axes)):
				axes[i] = 0
	

		if abs(axes[0]) <= 0.1:
			axes[0] = 0
	
		if abs(axes[1]) <= 0.1:
			axes[1] = 0

		if abs(axes[2]) <= 0.1:
			axes[2] = 0

		if abs(axes[3]) <= 0.1:
			axes[3] = 0


		axes[1] *= 2
		axes[3] *= math.pi

		mensaje = Twist2DStamped()
		mensaje.v = axes[0]
		mensaje.omega = axes[3]

		self.pub_control.publish(mensaje)


def main():
	rospy.init_node('test') #creacion y registro del nodo!
	
	# rosrun desafios_2022 template_cv.py 	
	obj = Template('args')

	rospy.spin()


if __name__ =='__main__':
	main()
