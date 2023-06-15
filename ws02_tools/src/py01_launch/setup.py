from setuptools import setup
from glob import glob

package_name = 'py01_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 文件相关配置
        ('share/' + package_name, glob("launch/py/*.launch.py")),
        ('share/' + package_name, glob("launch/xml/*.launch.xml")),
        ('share/' + package_name, glob("launch/yaml/*.launch.yaml")),
        # config 文件相关配置
        ('share/' + package_name, glob("config/*.yaml"))
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
        ],
    },
)
