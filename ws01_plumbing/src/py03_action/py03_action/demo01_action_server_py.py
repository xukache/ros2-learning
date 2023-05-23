"""
    需求：编写动作服务端，需要解析客户端提交的数字，遍历该数字并累加求和，最终结果响应回客户端，
    且请求响应过程中需要生成连续反馈。
    流程：
        1. 导包
        2. 初始化 ROS2 客户端
        3. 自定义节点类
            3-1. 创建动作服务端对象
            3-2. 处理提交的目标值（回调函数）---默认实现
            3-3. 处理取消请求（回调函数）---默认实现
            3-4. 生成连续反馈与最终响应（回调函数）
        4. 调用 spin 函数，并传入节点对象
        5. 资源释放
"""

# 1. 导包
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from base_interfaces_demo.action import Progress


# 3. 自定义节点类
class ProgressActionServer(Node):
    def __init__(self):
        super().__init__("progress_action_sever_node_py")
        self.get_logger().info('动作通信服务端启动了（Python）')
        # 3-1. 创建动作服务端对象
        """
            参数1：node 节点
            参数2：action_type 动作类型
            参数3：action_name 动作名称
            参数4：execute_callback 动作执行的回调函数
        """
        self.action_server_ = ActionServer(
            self,
            Progress,
            "get_sum",
            self.execute_callback
        )

    # 3-4. 生成连续反馈与最终响应（回调函数）
    def execute_callback(self, goal_handle):
        # 1. 生成连续反馈
        feedback_msg = Progress.Feedback()
        # 首先要获取目标值，然后遍历，遍历中进行累加，且每循环一次就计算进度，并作为连续反馈发布
        num = goal_handle.request.num
        sum = 0
        for i in range(1, num + 1):
            sum += i
            feedback_msg.progress = i / num
            self.get_logger().info(f'连续反馈：{feedback_msg.progress *100}%')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0)

        # 2. 响应最终结果
        goal_handle.succeed()
        result = Progress.Result()
        result.sum = sum

        self.get_logger().info(f'计算结果：{result.sum}')
        return result


def main(args=None):
    # 2. 初始化 ROS2 客户端
    rclpy.init(args=args)
    # 4. 调用 spin 函数，并传入节点对象
    rclpy.spin(ProgressActionServer())
    # 5. 资源释放
    rclpy.shutdown()


if __name__ == '__main__':
    main()
