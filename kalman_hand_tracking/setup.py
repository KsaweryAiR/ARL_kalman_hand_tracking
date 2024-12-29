from setuptools import find_packages, setup

package_name = 'kalman_hand_tracking'

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
    maintainer='ksawery',
    maintainer_email='85024534+KsaweryAiR@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_node = kalman_hand_tracking.camera_node:main",
            "drone_control_node = kalman_hand_tracking.drone_control_node:main",
        ],
    },
)
