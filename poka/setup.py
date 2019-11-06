from setuptools import find_packages
from setuptools import setup
import os
from glob import glob

package_name = 'poka'

setup(
    name=package_name,
    version='0.7.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='myasu',
    author_email='trihome@users.noreply.github.com',
    maintainer='myasu',
    maintainer_email='trihome@users.noreply.github.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: MIT License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Poka-Yoke System'
    ),
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpioout = node.GpioOut:main',
            'gpioin = node.GpioIn:main',
            'gpioi2c = node.GpioI2c:main',
            'gpiotest = node.GpioTest:main',
            'procconsole = node.ProcConsole:main',
            'procpoka = node.ProcPoka:main',
        ],
    },
)
