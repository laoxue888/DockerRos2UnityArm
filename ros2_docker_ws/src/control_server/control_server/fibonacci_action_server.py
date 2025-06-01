import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from panda_arm_msg.action import Fibonacci
import time

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        
        self.rate = self.create_rate(2)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # 初始化斐波那契序列
        sequence = [0, 1, 1]
        
        # 反馈消息
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = sequence
        
        # 检查目标是否有效
        if goal_handle.request.order <= 0:
            goal_handle.abort()
            result = Fibonacci.Result()
            result.sequence = sequence
            return result
        
        # 执行计算
        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i-1])
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            
            # 检查是否被取消
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            # 模拟长时间运行的任务
            # self.rate.sleep()
            time.sleep(0.5)  # 模拟延时，实际应用中可以使用 self.rate.sleep() 来控制频率

        # 完成目标
        goal_handle.succeed()
        
        # 设置结果
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info(f'Result: {result.sequence}')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()