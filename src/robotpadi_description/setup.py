from setuptools import setup
import os
from glob import glob

package_name = 'robotpadi_description'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
]

# Install models recursively
for root, dirs, files in os.walk('models'):
    if files:
        data_files.append(
            (
                os.path.join('share', package_name, root),
                [os.path.join(root, f) for f in files]
            )
        )

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
)