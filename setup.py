from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='massimo',
    maintainer_email='girardellimassimo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = px4_py.gui:main',
            'px4_py = px4_py.px4_py:main',
            'opti_to_px4 = px4_py.opti_to_px4:main',
        ],
    },
)
