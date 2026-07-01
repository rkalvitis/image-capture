from setuptools import find_packages, setup

setup(
    name='orbbec_camera_ros',
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roberts Kalvitis',
    maintainer_email='r.kalvitis@gmail.com',
    entry_points={
        'console_scripts': [
            'camera_node = orbbec_camera_ros.camera_node:main',
        ],
    },
)
