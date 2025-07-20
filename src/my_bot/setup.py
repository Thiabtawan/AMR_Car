from setuptools import setup

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[
        'lidar_node',
        'encoder_to_odom',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senseri',
    maintainer_email='you@example.com',
    description='AMR robot nodes and launch files',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_to_odom = encoder_to_odom:main',
            'lidar_node = lidar_node:main',
            'hw_launch = hw_full:main',
        ],
    },
)
