from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'led_state'
modules = "led_state/modules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, modules],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'led_state_launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='refro',
    maintainer_email='refro@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = led_state.led_controller:main',
            'leds = led_state.LED_STATE:main',
            'listener = led_state.controller:main',
        ],
    },
)
