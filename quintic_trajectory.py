import rospy
from geometry_msgs.msg import Twist

class QuinticTrajectoryNode:
    def __init__(self, T, rate_hz=10):
        self.T = T
        self.rate_hz = rate_hz
        self.pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        self.rate = rospy.Rate(rate_hz)

    def quintic_polynomial(self, t):
        # 5次の多項式の係数
        a0 = 0
        a1 = 0
        a2 = 0
        a3 = 10
        a4 = -15
        a5 = 6

        # 正規化時間
        tau = t / self.T

        # 位置
        s = a5 * tau**5 + a4 * tau**4 + a3 * tau**3 + a2 * tau**2 + a1 * tau + a0
        # 速度
        ds = (5 * a5 * tau**4 + 4 * a4 * tau**3 + 3 * a3 * tau**2 + 2 * a2 * tau + a1) / self.T
        # 加速度
        dds = (20 * a5 * tau**3 + 12 * a4 * tau**2 + 6 * a3 * tau + 2 * a2) / (self.T**2)

        return s, ds, dds

    def run(self):
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            t = (current_time - start_time).to_sec()

            if t < self.T:
                s, ds, dds = self.quintic_polynomial(t)
                move_cmd = Twist()
                move_cmd.linear.x = ds  # 多項式から計算した速度を使用
                self.pub.publish(move_cmd)
            else:
                self.pub.publish(Twist())  # ロボットを停止
                break

            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('quintic_trajectory_node')

    T = 10.0  # モーションの総時間
    node = QuinticTrajectoryNode(T)
    node.run()
