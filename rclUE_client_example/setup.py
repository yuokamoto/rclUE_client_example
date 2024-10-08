import os
from glob import glob

from setuptools import setup

package_name = 'rclUE_client_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rapyuta',
    maintainer_email='yuokamoto1988@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'conveyor_client = rclUE_client_example.conveyor_client:main',
                'spline_conveyor_client = rclUE_client_example.spline_conveyor_client:main',
                'ai_actor_client = rclUE_client_example.ai_actor_client:main',
                'tb3_client = rclUE_client_example.tb3_client:main',
                'warehouse_client = rclUE_client_example.warehouse_client:main'
        ],
    },
)
