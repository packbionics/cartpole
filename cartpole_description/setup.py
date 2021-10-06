from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'cartpole_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('robot/urdf/*')),
        (os.path.join('share', package_name), glob('rviz/*')),
        (os.path.join('share', package_name), glob('meshes/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasonx',
    maintainer_email='59701038+JChunX@users.noreply.github.com',
    description='Cartpole descriptions and simulations',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_state_publisher = cartpole_description.demo_state_publisher:main'
        ],
    },
)
