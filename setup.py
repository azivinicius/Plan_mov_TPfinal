from setuptools import setup
import os
from glob import glob

package_name = 'SPH'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Include all world files (.world and .sdf)
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', 'SPH', 'worlds', 'meshes'), glob('worlds/meshes/*.stl')),
        
        # 3. Include map files (.yaml and .pgm) used by your occupancy_grid node
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        
        # 4. Include robot model files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azivinicius',
    maintainer_email='azivinicius@todo.todo',
    description='SPH Swarm Simulation Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Your nodes must be registered here to be executable via 'ros2 run'
            'occupancy_grid = SPH.occupancy_grid:main',
            'diff_sph = SPH.diff_sph:main',
            'basic = SPH.basic_potential:main',
        ],
    },
)