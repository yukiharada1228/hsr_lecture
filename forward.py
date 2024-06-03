#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

if __name__=="__main__":
    rospy.init_node('move_forward_node')

    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)

    move_cmd = Twist()
    move_cmd.linear.x = 0.1  # 1メートル進むのに必要な速度 (0.1メートル/秒)

    rate = rospy.Rate(10)  # 10Hzで送信

    # 1メートル進むための時間 (10秒)
    duration = rospy.Duration(10.0)

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        if current_time - start_time < duration:
            pub.publish(move_cmd)
        else:
            # 指定した時間が経過したら停止する
            pub.publish(Twist())
            break

        rate.sleep()
