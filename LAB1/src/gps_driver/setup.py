from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gps_msgs', 'std_msgs'],
    zip_safe=True,
    maintainer='jackg',
    maintainer_email='gladowsky.j@northeastern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_driver = gps_driver.driver:main'
        ],
    },
)