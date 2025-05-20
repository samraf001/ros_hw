from setuptools import setup

package_name = 'ros_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='urafiq',
    maintainer_email='urafiq@jhfletcher.com',
    description='3-DOF sensor simulation with ROS2 Python and custom service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_service_server = ros_hw.service_server:main',  # Update to use a single entry point for both
            'data_client = ros_hw.data_client:main',  # Data Client
        ],
    },
)
