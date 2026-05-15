from setuptools import find_packages, setup

package_name = 'ros_lecture_group1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ros_lecture_group1.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_lecture_group1',
    maintainer_email='todo@example.com',
    description='ROS2 package skeleton for the group 1 lecture project.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sm_main = ros_lecture_group1.sm_main:main',
            'task_node = ros_lecture_group1.task_node:main',
        ],
    },
)
