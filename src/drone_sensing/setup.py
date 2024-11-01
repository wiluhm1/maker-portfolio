from setuptools import find_packages, setup

package_name = 'drone_sensing'

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
    maintainer='zuri2102',
    maintainer_email='zuriye45@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dc_node = drone_sensing.downward_cam:main",
            "stereo_node = drone_sensing.stereo_cam:main"
        ],
    },
)
