#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy
import math
from duckietown_msgs.msg import Twist2DStamped

def test(pub):
        def callback(msg):
                axes = list(msg.axes)
                buttons = list(msg.buttons)
                B = buttons[1]

                if B == 1:
                        for i in range(len(axes)):
                                axes[i] = 0

                        for i in range(len(buttons)):
                                buttons[i] = 0



                if abs(axes[0]) <= 0.1:
                        axes[0] = 0

                if abs(axes[1]) <= 0.1:
                        axes[1] = 0

                if abs(axes[3]) <= 0.1:
                        axes[3] = 0

                if abs(axes[4]) <= 0.1:
                        axes[4] = 0

                axes[1] *= 2
                axes[3] *= 2 * math.pi * 0.5

                # Extra

                print("Eje 0: " + str(axes[0]))

                # Extra:
                print("Eje 3: " + str(axes[3]))
                mensaje = Twist2DStamped()
                mensaje.v = axes[0]
                mensaje.omega = axes[3]
                pub.publish(mensaje)

        return callback
        
def main():
        rospy.init_node("listener", anonymous=True)
        pub= rospy.Publisher("duckiebot/wheels_driver_node/car_cmd", Twist2DStamped )
        rospy.Subscriber("/duckiebot/joy", Joy, test(pub))

        rospy.spin()

if __name__ =='__main__':
        main()
