from setuptools import find_packages, setup

package_name = 'robotics_fundamentals_ros_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahb',
    maintainer_email='andreas.bihlmaier@gmx.de',
    description='ROSCon DE 2023 Learning Robotics Fundamentals with ROS 2 and modern Gazebo',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gentle_twist = robotics_fundamentals_ros_gazebo.gentle_twist:main',
        ],
    },
)
