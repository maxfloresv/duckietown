#!/usr/bin/env python

import rospy 
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

def callback(msg):
	axes = list(msg.axes)
	buttons = list(msg.buttons)
	B = buttons[1]

  	if B == 1:
		for i in range(len(axes)):
			axes[i] = 0

	  	for i in range(len(buttons)):
		  	buttons[i] = 0
	
	axes[0] *= 10
	axes[1] *= math.pi * 0.5

	# Extra:
	axes[3] = 0.5 * axes[3] + 0.5

	print("Eje 0: " + str(axes[0]))
	print("Eje 1: " + str(axes[1]))

	# Extra:
	print("Eje 3: " + str(axes[3]))

def main():
	rospy.init_node("listener", anonymous=True)

	rospy.Subscriber("/duckiebot/joy", Joy, callback)

	rospy.spin()

if __name__ == '__main__':
	main()
