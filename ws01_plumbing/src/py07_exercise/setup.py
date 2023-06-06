import os
from glob import glob
from setuptools import setup

package_name = 'py07_exercise'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xukai',
    maintainer_email='2416695482@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "exe01_pub_sub_py = py07_exercise.exe01_pub_sub_py:main",
            "exe02_server_py = py07_exercise.exe02_server_py:main",
            "exe03_client_py = py07_exercise.exe03_client_py:main"
        ],
    },
)
