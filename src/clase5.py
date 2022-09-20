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
		
		image_out = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		lower_limit = np.array([25, 130, 130])
		upper_limit = np.array([35, 255, 255])
    
		mask = cv2.inRange(image_out, lower_limit, upper_limit)
		
    		kernel = np.ones((5, 5), np.uint8)
    		img_out = cv2.erode(mask, kernel, iterations=3)
    		img_out = cv2.dilate(img_out, kernel, iterations=3)
    
    		img_masked = cv2.bitwise_and(image, image, mask=mask)
    
    		_, contours, hierarchy = cv2.findContours(img_out, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    		min_area = 5
    
    		for val in contours:
      			x, y, w, h = cv2.boundingRect(val)
      			if w*h > min_area:
        			cv2.rectangle(image, (x+w, y+h), (x, y), (0, 0, 0), 2)
        
    		msg = bridge.cv2_to_imgmsg(image, "bgr8")
		self.pub.publish(msg)


def main():
	rospy.init_node('test') 
  
	obj = Template('args')
  
	rospy.spin()


if __name__ =='__main__':
	main()
