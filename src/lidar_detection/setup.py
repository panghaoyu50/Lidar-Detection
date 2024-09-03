from setuptools import find_packages, setup

package_name = 'lidar_detection'

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
    maintainer='pang',
    maintainer_email='panghaoyu50@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bounding_box_filter_node = lidar_detection.bounding_box_filter:main',
            'ground_removal_node = lidar_detection.ground_removal:main',
            'clustering_node = lidar_detection.clustering:main',
        ],
    },
)
