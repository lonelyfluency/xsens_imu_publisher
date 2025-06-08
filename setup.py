from setuptools import setup

package_name = 'xsens_imu_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dada',
    maintainer_email='lonelyfluency@gmail.com',
    description='ROS 2 node to publish IMU data from Xsens MTi using XDA.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'xsens_node = xsens_imu_publisher.xsens_node:main',
        ],
    },
)
