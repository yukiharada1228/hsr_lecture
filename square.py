#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math

class SquareDrawer:
    def __init__(self, linear_speed, angular_speed):
        rospy.init_node('move_square_node')
        self.pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        self.move_cmd = Twist()
        self.linear_speed = linear_speed  # 直進速度 (m/s)
        self.angular_speed = angular_speed  # 旋回速度 (rad/s)
        self.rate = rospy.Rate(10)  # 10Hzで送信

    def move(self, linear, angular, duration):
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < duration:
            self.move_cmd.linear.x = linear
            self.move_cmd.angular.z = angular
            self.pub.publish(self.move_cmd)
            self.rate.sleep()

    def stop(self):
        self.move(0.0, 0.0, rospy.Duration(1.0))

    def draw_square(self):
        duration_straight = self.calculate_duration_straight()  # 直進時間 (秒)
        duration_turn = self.calculate_duration_turn()  # 旋回時間 (秒)
        for _ in range(4):  # 四回繰り返す
            # 直進
            self.move(self.linear_speed, 0.0, duration_straight)

            # 停止
            self.stop()

            # 旋回
            self.move(0.0, self.angular_speed, duration_turn)

            # 停止
            self.stop()

    def calculate_duration_straight(self):
        # 直進時間の計算
        return rospy.Duration(1/self.linear_speed)

    def calculate_duration_turn(self):
        # 旋回時間の計算
        return rospy.Duration((math.pi/2)/self.angular_speed)

if __name__=="__main__":
    linear_speed = 0.1  # 直進速度 (m/s)
    angular_speed = 0.2  # 旋回速度 (rad/s)
    square_drawer = SquareDrawer(linear_speed, angular_speed)
    square_drawer.draw_square()
