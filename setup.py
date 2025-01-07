import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'hex_toolkit_echo_plus'


def get_files(tar: str, src: str):
    all_paths = glob(f'{src}/*')

    data_files = []
    for path in all_paths:
        if os.path.isfile(path):
            data_files.append((tar, [path]))
        elif os.path.isdir(path):
            sub_files = get_files(f'{tar}/{os.path.basename(path)}', path)
            data_files.extend(sub_files)

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *get_files('share/' + package_name + '/launch/ros2', "launch/ros2"),
        *get_files('share/' + package_name + '/config/ros2', "config/ros2"),
        *get_files('share/' + package_name + '/urdf', "urdf"),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='847235539@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "mpc_track = hex_toolkit_echo_plus.mpc_track:main",
        ],
    },
)
