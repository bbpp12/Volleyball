/**
 * Robocon排球赛ROS2下位机节点：订阅球点x/y/yaw数据，解析后对接运动控制
 * 通讯话题：/robocon/ball_point
 * 消息类型：std_msgs/msg/Float32MultiArray [x, y, yaw]
 * 数据超时：100ms（未收到数据则进入安全状态）
 * 单位：x/y → mm，yaw → °
 * 适配：STM32/树莓派/Jetson Nano（ROS2 Humble C++17）
 */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>

// 命名空间简写（减少代码冗余，ROS2工程化规范）
using namespace std::chrono_literals;
using std::placeholders::_1;

// 全局变量：存储球点数据（供运动控制函数调用）
float g_ball_x = 0.0f;
float g_ball_y = 0.0f;
float g_ball_yaw = 0.0f;
bool g_data_valid = false;          // 数据有效标志
rclcpp::Time g_last_recv_time;      // 最后一次接收数据时间（超时判断）
const int DATA_TIMEOUT_MS = 100;    // 数据超时时间：100ms（抗干扰关键）

// 定义订阅节点类（继承ROS2的rclcpp::Node类）
class BallPointSubscriber : public rclcpp::Node
{
public:
    // 构造函数：初始化节点、创建订阅者、创建运动控制定时器
    BallPointSubscriber() : Node("lower_ball_point_sub")
    {
        // 创建订阅者：订阅球点话题，绑定回调函数
        ball_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robocon/ball_point",
            10,
            std::bind(&BallPointSubscriber::ball_point_callback, this, _1)
        );

        // 创建运动控制定时器：30Hz（与上位机发布频率同步）
        control_timer_ = this->create_wall_timer(
            33ms,  // 30Hz ≈ 33毫秒
            std::bind(&BallPointSubscriber::motion_control_task, this)
        );

        // 初始化最后接收数据时间
        g_last_recv_time = this->get_clock()->now();
        // 启动日志
        RCLCPP_INFO(this->get_logger(), "【下位机订阅节点】已启动，等待球点数据...");
    }

private:
    // 订阅者对象（类成员）
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ball_sub_;
    // 运动控制定时器对象（类成员）
    rclcpp::TimerBase::SharedPtr control_timer_;

    /**
     * @brief 球点数据回调函数：订阅到数据时触发，解析并校验数据
     * @param msg 订阅到的ROS2标准消息
     */
    void ball_point_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // 1. 校验数据长度：必须包含x/y/yaw三个浮点值
        if (msg->data.size() < 3)
        {
            RCLCPP_WARN(this->get_logger(), "【接收错误】数据长度无效，实际：%lu，要求：3", msg->data.size());
            g_data_valid = false;
            return;
        }

        // 2. 解析球点数据，更新全局变量
        g_ball_x = msg->data[0];
        g_ball_y = msg->data[1];
        g_ball_yaw = msg->data[2];

        // 3. 更新最后接收时间，标记数据有效
        g_last_recv_time = this->get_clock()->now();
        g_data_valid = true;

        // 4. 调试日志（DEBUG级别，发布时可注释）
        RCLCPP_DEBUG(
            this->get_logger(),
            "【接收成功】x=%.1fmm, y=%.1fmm, yaw=%.1f°",
            g_ball_x, g_ball_y, g_ball_yaw
        );
    }

    /**
     * @brief 运动控制任务：30Hz定时执行，对接实际硬件控制逻辑
     * @note 无有效数据/数据超时时，进入安全状态（电机停止、舵机归位）
     */
    void motion_control_task()
    {
        // 1. 数据超时判断：超过100ms未收到新数据，置为无效
        auto current_time = this->get_clock()->now();
        auto time_duration = current_time - g_last_recv_time;
        if (time_duration > DATA_TIMEOUT_MS * 1ms)
        {
            g_data_valid = false;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "【运动控制】数据超时，进入安全状态");
            return;
        }

        // 2. 无有效数据时，直接返回（安全状态）
        if (!g_data_valid)
        {
            return;
        }

        // -------------------------- 核心替换区 --------------------------
        // 此处添加实际运动控制逻辑，使用全局变量g_ball_x/g_ball_y/g_ball_yaw
        // 示例：
        // 1. 对接STM32：通过串口/CAN总线发送控制指令；
        // 2. 对接电机驱动：调用电机库设置转速/位置；
        // 3. 对接舵机：调用舵机库设置偏航角；
        // 注意：单位与上位机一致（x/y→mm，yaw→°）
        // ----------------------------------------------------------------
    }
};

// 节点主入口
int main(int argc, char * argv[])
{
    // 1. 初始化ROS2 C++客户端
    rclcpp::init(argc, argv);
    // 2. 创建节点实例并自旋运行（ROS2核心，处理回调和定时器）
    rclcpp::spin(std::make_shared<BallPointSubscriber>());
    // 3. 销毁节点，关闭ROS2客户端
    rclcpp::shutdown();
    return 0;
}
