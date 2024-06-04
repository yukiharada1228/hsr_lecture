#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class MoveForwardNode:
    def __init__(self, duration_sec, speed, rate_hz=10):
        self.duration_sec = duration_sec
        self.speed = speed
        self.rate_hz = rate_hz

        rospy.init_node('move_forward_node')
        self.pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        self.rate = rospy.Rate(rate_hz)
        self.move_cmd = Twist()
        self.move_cmd.linear.x = speed  # 指定された速度 (メートル/秒)

    def run(self):
        duration = rospy.Duration(self.duration_sec)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if current_time - start_time < duration:
                self.pub.publish(self.move_cmd)
            else:
                # 指定した時間が経過したら停止する
                self.pub.publish(Twist())
                break

            self.rate.sleep()

if __name__ == "__main__":
    # 1メートル進むために必要な時間と速度
    duration_sec = 10.0  # 秒
    speed = 0.1  # メートル/秒

    node = MoveForwardNode(duration_sec, speed)
    node.run()
