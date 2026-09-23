from glob import glob

from setuptools import find_packages, setup

package_name = 'muto_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Aguadze',
    maintainer_email='aguadzemic@gmail.com',
    description='Gamepad drive and camera pan/tilt for the Muto hexapod',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_node = muto_teleop.teleop_node:main',
        ],
    },
)
