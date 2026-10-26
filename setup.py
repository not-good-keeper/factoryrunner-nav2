from glob import glob

from setuptools import find_packages, setup

package_name = 'factoryrunner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nethi Kushala Kumar',
    maintainer_email='nethi.kushaal@gmail.com',
    description='TurtleBot3 + Nav2 factory logistics demo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = factoryrunner.mission_controller:main',
        ],
    },
)
