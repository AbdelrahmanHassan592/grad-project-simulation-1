from setuptools import setup
from pathlib import Path
package_name = 'my_car_description'
data = [('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md'])]
for folder in ('urdf', 'launch', 'worlds', 'models', 'config', 'rviz', 'scripts'):
    for directory in [Path(folder), *[p for p in Path(folder).rglob('*') if p.is_dir() and '__pycache__' not in p.parts]]:
        data.append(('share/' + package_name + '/' + str(directory),
                     [str(p) for p in directory.iterdir() if p.is_file() and p.suffix not in ('.pyc', '.pyo')]))
setup(name=package_name, version='1.0.0', packages=[package_name], data_files=data,
      install_requires=['setuptools'], zip_safe=True, maintainer='Abdelrahman',
      maintainer_email='abdelrahman@abdelrahman-HP-ZBook-15-G5.local', description='Primitive skid-steer car for Gazebo Harmonic',
      license='Apache-2.0', entry_points={'console_scripts': [
          'check_world = my_car_description.world_check:main',
          'keyboard_drive = my_car_description.keyboard_drive:main',
          'test_sim = my_car_description.sim_test:main']})
