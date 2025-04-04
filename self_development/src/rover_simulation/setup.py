import os
from setuptools import find_packages, setup
from glob import glob
package_name = 'rover_simulation'

data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name, ["package.xml"]))

def package_files(directory, data_files):
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            data_files.append(("share/" + package_name + "/" + path, glob(path + "/**/*.*", recursive=True)))
    return data_files

data_files = package_files('launch/', data_files)
data_files = package_files('urdf/', data_files)
data_files = package_files('meshes/', data_files)
data_files = package_files('config/', data_files)
data_files = package_files('world/', data_files)
data_files = package_files('models/mars_terrain/', data_files)
data_files = package_files('models/mars_terrain/meshes/', data_files)
data_files = package_files('models/mars_terrain/meshes/scripts/', data_files)
data_files = package_files('models/mars_terrain/meshes/textures/', data_files)


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tora',
    maintainer_email='tiger.tora1210@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
