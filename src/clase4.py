#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self.callback)
		self.pub = rospy.Publisher("/duckiebot/camera_node/image/test", Image)

	def callback(self,msg):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(msg, "bgr8")
		
		image_out = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
		
		lower_limit = np.array([49, 168, 162])
		upper_limit = np.array([3, 252, 240])
		mask = cv2.inRange(image_out, lower_limit, upper_limit)
		
		msg = bridge.cv2_to_imgmsg(image_out, "bgr8")
		self.pub.publish(msg)


def main():
	rospy.init_node('test') #creacion y registro del nodo!
	
	# rosrun desafios_2022 template_cv.py 	
	obj = Template('args')

	rospy.spin()


if __name__ =='__main__':
	main()
