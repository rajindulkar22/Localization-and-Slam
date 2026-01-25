from setuptools import find_packages, setup

package_name = 'tb4_perception'

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
    maintainer='raj',
    maintainer_email='raj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_reader = tb4_perception.lidar_reader:main',
            'safety_visualizer = tb4_perception.safety_visualizer:main',
            'mapping_node = tb4_perception.mapping_node:main',
            'particle_filter = tb4_perception.particle_filter:main'
        ],
    },
)
