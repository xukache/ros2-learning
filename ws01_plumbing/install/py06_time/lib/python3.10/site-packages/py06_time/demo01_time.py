# 1. 导包
import rclpy
from rclpy.node import Node
import threading
from rclpy.time import Time
from rclpy.time import Duration


# 3. 定义节点类
class MyNode(Node):
    def __init__(self):
        super().__init__('time_node_py')
        # 调用 Rate 函数
        # self.demo_rate()
        # 调用 Time 函数
        # self.demo_time()
        # 调用 Duration 函数
        # self.demo_duration()
        # 调用运算函数
        self.demo_opt()

    # 演示 Rate 的使用
    def demo_rate(self):
        # 1. 创建 Rate 对象
        """
        create_rate:
        参数1：frequency：Rate 运行的频率(Hz)  1Hz周期是1秒
        """
        self.rate = self.create_rate(1.0)
        # 2. 调用 sleep 函数  ---- 导致程序阻塞
        # 解决方法一：使用 time.sleep() 替换
        # while rclpy.ok():
        #     self.get_logger().info("+++++++++++++++++++++")
        #     self.rate.sleep()

        # # 解决方法二：创建子线程实现运行频率控制
        thread = threading.Thread(target=self.do_some)
        thread.start()

    def do_some(self):
        while rclpy.ok():
            self.get_logger().info("+++++++++++++++++++++")
            self.rate.sleep()

    # 演示 Time 的使用
    def demo_time(self):
        # 1. 创建 Time 对象
        """
        Time: 
            参数1：seconds(秒): int
            表示创建一个时间戳，设置为从 UNIX 纪元开始 seconds 秒后的时间
            参数2：nanoseconds(纳秒)：int
            表示创建一个时间戳，设置为从 UNIX 纪元开始 nanoseconds 纳秒后的时间
        """
        # 这个时间戳表示从 UNIX 纪元开始，经过了 5 秒和 500000000 纳秒,
        # 该时间点距离 UNIX 纪元的时间为 5.5 秒。
        t1 = Time(
            seconds=5,
            nanoseconds=500000000)
        right_now = self.get_clock().now()
        # 2. 调用 Time 对象的函数
        self.get_logger().info("t1: s = %.2f, ns = %d" %
                               (t1.seconds_nanoseconds()[0], t1.seconds_nanoseconds()[0]))
        self.get_logger().info("right_now: s = %.2f, ns = %d" %
                               (right_now.seconds_nanoseconds()[0], right_now.seconds_nanoseconds()[0]))
        self.get_logger().info("t1: ns = %d" % t1.nanoseconds)
        self.get_logger().info("right_now: ns = %d" % right_now.nanoseconds)

    # 演示 Duration 的使用
    def demo_duration(self):
        # 1. 创建 Duration 对象
        """
        Duration: 
            参数1：seconds(秒): int
            表示创建一个时间间隔，设置为 seconds 秒
            参数2：nanoseconds(纳秒)：int
            表示创建一个时间间隔，设置为 nanoseconds 纳秒
        """
        # 这个时间间隔的值为 (10 + 0.8) 秒
        du1 = Duration(seconds=10, nanoseconds=800000000)
        # 2. 调用 Duration 对象的函数
        self.get_logger().info("du1: ns = %d" % du1.nanoseconds)

    # 演示运算
    def demo_opt(self):
        t1 = Time(seconds=20)
        t2 = Time(seconds=15)

        du1 = Duration(seconds=7)
        du2 = Duration(seconds=13)
        # time 比较
        self.get_logger().info("t1 >= t2? %d" % (t1 >= t2))  # 1
        self.get_logger().info("t1 < t2? %d" % (t1 < t2))  # 0
        # 数学运算
        t3 = t1 - t2
        t4 = t1 + du1
        t5 = t1 - du1

        self.get_logger().info("t3 = t1 - t2 = %d" % t3.nanoseconds)  # 5
        self.get_logger().info("t4 = t1 + du1 = %d" % t4.nanoseconds)  # 27
        self.get_logger().info("t5 = t1 - du1 = %d" % t5.nanoseconds)  # 13

        # duration 比较
        self.get_logger().info("du1 >= du2? %d" % (du1 >= du2))  # 0
        self.get_logger().info("du1 < du2? %d" % (du1 < du2))  # 1


def main(args=None):
    # 2. 初始化 ROS2 客户端
    rclpy.init(args=args)
    # 4. 调用 spin 函数，并传入节点对象
    time_node = MyNode()
    rclpy.spin(time_node)
    # 5. 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()
