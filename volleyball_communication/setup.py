from setuptools import setup
import os
from glob import glob

# 包名，与功能包名一致
package_name = 'volleyball_communication'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ROS2资源索引配置
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 安装package.xml
        ('share/' + package_name, ['package.xml']),
        # 安装launch目录下的所有启动文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@xxx.com',
    description='Robocon排球赛ROS2上位机-下位机球点通讯包',
    license='MIT',
    tests_require=['pytest'],
    # Python节点入口：执行ros2 run时调用的命令
    entry_points={
        'console_scripts': [
            "ball_point_publisher = volleyball_communication.ball_point_publisher:main",
        ],
    },
)
