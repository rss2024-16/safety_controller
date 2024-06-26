from setuptools import setup
import os
from glob import glob

package_name = 'safety_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', "safety_controller/params.yaml", "safety_controller/robotparams.yaml"]),
        (os.path.join('share', package_name), glob('launch/*launch.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='ellen660@mit.edu',
    description='Safety controller for lab 3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_controller = safety_controller.safety_controller:main',
            'sc_max = safety_controller.sc_max:main'
        ],
    },
)
