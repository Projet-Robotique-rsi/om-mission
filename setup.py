from setuptools import find_packages, setup

package_name = 'om_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehdi-Robotics',
    maintainer_email='yasserdjani0@gmail.com',
    description='High-level mission planner for pick and place',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_planner = om_mission.mission_planner:main',
        ],
    },
)
