#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.rate = rospy.Rate(10)

    def callback(self, data):
        twist = Twist()
        right = min(min(data.ranges[0:20]), 3.0)
        front = min(min(data.ranges[340:360] + data.ranges[0:20]), 3.0)

        if front < 0.5:
            twist.angular.z = 0.5
        elif right < 0.5:
            twist.linear.x = 0.2
            twist.angular.z = 0.1
        else:
            twist.linear.x = 0.3
            twist.angular.z = -0.2

        self.cmd_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        WallFollower().run()
    except rospy.ROSInterruptException:
        pass
