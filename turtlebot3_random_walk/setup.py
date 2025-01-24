from os import path
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_random_walk'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luizcarloscf',
    maintainer_email='luiz.cosmi@edu.ufes.br',
    description='Turtlebot3 random walk',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_random_walk = turtlebot3_random_walk.main:main'
        ],
    },
)
