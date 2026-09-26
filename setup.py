from setuptools import find_packages, setup

package_name = 'bidirectional_motor_test'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['tests', 'test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor_test.launch.py']),
        ('share/' + package_name + '/config', ['config/motor_test.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssybh2',
    maintainer_email='ssybh2@users.noreply.github.com',
    description='DJI RC gated sinusoidal DSHOT600 reversal and thrust timing.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_test = bidirectional_motor_test.motor_test_node:main',
        ],
    },
)
