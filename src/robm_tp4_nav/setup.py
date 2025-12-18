import os
from glob import glob
from setuptools import setup

package_name = 'robm_tp4_nav'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch','*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vincent Drevelle',
    maintainer_email='vincent.drevelle@univ-rennes.fr',
    description='Outils et squelettes de code pour le TP4/TP5 de ROBM',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_heading = robm_tp4_nav.control_heading:main',
            'move = robm_tp4_nav.move:main',
            'hard_trophy = robm_tp4_nav.hard_trophy:main',
            'obstacle = robm_tp4_nav.obstacle_avoid:main',
            'simple_trophy = robm_tp4_nav.simple_trophy:main',
            'servo = robm_tp4_nav.servo:main',
            'gripper = robm_tp4_nav.gripper:main',
            'offroad = robm_tp4_nav.offroad:main',
        ],
    },
)
