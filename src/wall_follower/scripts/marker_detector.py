#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class MarkerDetector:
    def __init__(self):
        rospy.init_node('marker_detector')
        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)

    def callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        lower = (0, 100, 100)    # Red color lower bound
        upper = (10, 255, 255)   # Red color upper bound
        mask = cv2.inRange(hsv, lower, upper)

        if cv2.countNonZero(mask) > 500:
            rospy.loginfo("Marker detected! Stopping...")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

if __name__ == '__main__':
    try:
        MarkerDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
