from glob import glob
from setuptools import setup

package_name = 'sound_play_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/sound', glob('sound/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ok',
    maintainer_email='unknown@example.com',
    description='sound play package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sound_play = sound_play_pkg.sound_play:main',
        ],
    },
)
