from setuptools import setup

package_name = 'cartpole_control'

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
    maintainer_email='jasonxie(at)live(dot)unc(dot)edu',
    description='Cartpole swingup + balancing controllers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'cartpole_teleop_key = cartpole_control.cartpole_teleop_key:main',
            'cartpole_test_server = cartpole_control.cartpole_test_server:main',
            'cartpole_swingup = cartpole_control.cartpole_swingup:main'
        ],
    },
)
