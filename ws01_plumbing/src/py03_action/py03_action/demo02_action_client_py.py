"""  
    需求：编写动作客户端实现，可以提交一个整型数据到服务端，并处理服务端的连续反馈以及最终返回结果。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建动作客户端；
            3-2.发送请求；
            3-3.处理目标发送后的反馈；
            3-4.处理连续反馈；
            3-5.处理最终响应。
        4.调用spin函数，并传入节点对象；
        5.释放资源。

"""
# 1.导包；
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from base_interfaces_demo.action import Progress

# 3.定义节点类；
class ProgressActionClient(Node):

    def __init__(self):
        super().__init__('progress_action_client')
        # 3-1.创建动作客户端；
        self._action_client = ActionClient(self, Progress, 'get_sum')

    def send_goal(self, num):
        # 3-2.发送请求；
        goal_msg = Progress.Goal()
        goal_msg.num = num
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # 3-3.处理目标发送后的反馈；
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('请求被拒绝')
            return

        self.get_logger().info('请求被接收，开始执行任务！')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # 3-5.处理最终响应。
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('最终计算结果：sum = %d' % result.sum)
        # 5.释放资源。
        rclpy.shutdown()

    # 3-4.处理连续反馈；
    def feedback_callback(self, feedback_msg):
        feedback = (int)(feedback_msg.feedback.progress * 100)
        self.get_logger().info('当前进度: %d%%' % feedback)


def main(args=None):

    # 2.初始化 ROS2 客户端；
    rclpy.init(args=args)
    # 4.调用spin函数，并传入节点对象；

    action_client = ProgressActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

    # rclpy.shutdown()


if __name__ == '__main__':
    main()