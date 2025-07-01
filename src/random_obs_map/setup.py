from setuptools import setup

import os
from glob import glob

package_name = 'random_obs_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加.world文件到安装目录
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # 如果你还有其他需要安装的目录，可以继续添加
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunfeibi',
    maintainer_email='2277241439@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate = random_obs_map.generate:training_loop',
        ],
    },
)
