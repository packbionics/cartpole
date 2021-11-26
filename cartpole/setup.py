from setuptools import setup

package_name = 'cartpole'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasonx',
    maintainer_email='59701038+JChunX@users.noreply.github.com',
    description='Main package for cartpole',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartpole_main = cartpole.cartpole_main:main',
        ],
    },
)
