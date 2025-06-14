from setuptools import setup

package_name = 'linear_regression_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/launch', [
            'launch/regression_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wonil Choi',
    maintainer_email='youremail@example.com',
    description='Linear regression ROS2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'regression_node = linear_regression_pkg.regression_node:main',
            'result_subscriber = linear_regression_pkg.result_subscriber:main',
        ],
    },
)
