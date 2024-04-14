import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roslab'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def add_share_dir(dir_path: str):
    global data_files
    data_files += [(os.path.join('share', package_name, os.path.split(path)[0]), [path]) for path in glob(f'{dir_path}/**', recursive=True) if not os.path.isdir(path)]

add_share_dir('launch')
add_share_dir('config')
add_share_dir('meshes')
add_share_dir('robots')
add_share_dir('worlds')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='met4000',
    maintainer_email='nathsharwor@hotmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
