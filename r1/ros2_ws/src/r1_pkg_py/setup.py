from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'r1_pkg_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                # ✅ INSTALL APRILTAG FILES
        (
            os.path.join('share', package_name, 'apriltag'),
            glob('r1_pkg_py/apriltag/*.png')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='dararithy123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gui_node = r1_pkg_py.gui_node:main',
        ],
    },
)
