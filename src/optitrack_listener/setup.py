from setuptools import find_packages, setup

package_name = 'optitrack_listener'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/optitrack.launch.py',
            'launch/record.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luca Crupi',
    maintainer_email='luca.crupi@icloud.com',
    description='Minimal subscriber for /optitrack/<rigid_body> topics.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'listener = optitrack_listener.listener:main',
            'mock_pose_publisher = optitrack_listener.mock_pose_publisher:main',
        ],
    },
)
