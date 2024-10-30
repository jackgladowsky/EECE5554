from setuptools import setup

package_name = 'imu_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='IMU driver node',
    license='Your license declaration',
    entry_points={
        'console_scripts': [
            'imu_driver = imu_driver.driver:main',  # Points to driver.py main function
        ],
    },
)
