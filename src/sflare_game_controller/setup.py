from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'sflare_game_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'sflare_game_controller.PallasSDK': ['*.so'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='Sflare机械臂小游戏下位控制节点。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_node = sflare_game_controller.imitate_node:main',
        ],
    },
)
