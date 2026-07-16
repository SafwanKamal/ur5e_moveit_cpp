# from setuptools import find_packages, setup

# package_name = 'ur5e_toolheads'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='desktop',
#     maintainer_email='desktop@todo.todo',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ur5e_toolheads'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'meshes'),
            glob('meshes/*'),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='desktop',
    maintainer_email='desktop@example.com',
    description='Custom UR5e tool-head descriptions',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
