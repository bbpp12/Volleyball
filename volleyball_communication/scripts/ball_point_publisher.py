#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robocon排球赛ROS2上位机节点：发布球点x/y/yaw数据
通讯话题：/robocon/ball_point
消息类型：std_msgs/msg/Float32MultiArray [x, y, yaw]
发布频率：30Hz（适配视觉识别与运动控制实时性）
单位：x/y → mm，yaw → °
"""
import rclpy
from rclpy.node import Node
from rclpy.timer import Rate
from std_msgs.msg import Float32MultiArray

# 定义发布节点类（继承ROS2的Node类）
class BallPointPublisher(Node):
    def __init__(self):
        # 初始化节点，节点名全局唯一
        super().__init__("upper_ball_point_pub")
        # 创建发布者：话题名/消息类型/队列大小
        self.ball_pub = self.create_publisher(
            msg_type=Float32MultiArray,
            topic="/robocon/ball_point",
            qos_profile=10
        )
        # 设置30Hz发布频率
        self.pub_rate = Rate(30, self.get_clock())
        # 启动日志
        self.get_logger().info("【上位机发布节点】已启动，开始发布球点数据...")

    def publish_ball_data(self):
        """核心发布逻辑：循环获取球点并发布"""
        while rclpy.ok():
            # -------------------------- 核心替换区 --------------------------
            # 此处替换为视觉模块的实际球点检测结果
            # 建议：封装视觉识别函数，在此处调用获取x/y/yaw
            ball_x = 185.6  # 球点x坐标，单位mm
            ball_y = 242.3  # 球点y坐标，单位mm
            ball_yaw = 38.5 # 球点偏航角yaw，单位°
            # ----------------------------------------------------------------

            # 打包数据为ROS2标准浮点数组消息
            ball_msg = Float32MultiArray()
            ball_msg.data = [ball_x, ball_y, ball_yaw]
            # 发布数据
            self.ball_pub.publish(ball_msg)
            # 调试日志（DEBUG级别，可通过ros2 run --log-level DEBUG开启）
            self.get_logger().debug(
                f"【发送球点】x={ball_x:.1f}mm, y={ball_y:.1f}mm, yaw={ball_yaw:.1f}°"
            )
            # 按30Hz频率休眠
            self.pub_rate.sleep()

# 节点主入口
def main(args=None):
    # 初始化ROS2 Python客户端
    rclpy.init(args=args)
    # 创建节点实例
    pub_node = BallPointPublisher()
    try:
        # 执行发布逻辑
        pub_node.publish_ball_data()
    except KeyboardInterrupt:
        # 捕获Ctrl+C中断，优雅退出
        pub_node.get_logger().warn("【上位机发布节点】用户强制中断！")
    except Exception as e:
        # 捕获其他异常，打印致命日志
        pub_node.get_logger().fatal(f"【上位机发布节点】运行出错：{str(e)}")
    finally:
        # 销毁节点，关闭ROS2客户端
        pub_node.destroy_node()
        rclpy.shutdown()

# 脚本直接运行时调用主入口
if __name__ == "__main__":
    main()
