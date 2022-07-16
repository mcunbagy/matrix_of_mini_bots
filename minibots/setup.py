from setuptools import setup

package_name = 'minibots'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/supervisor_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/supervisor2_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/supervisor3_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/supervisor4_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/supervisor5_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/supervisor6_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/supervisor7_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/supervisor8_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world2.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world3.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world4.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world5.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world6.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world7.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world8.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/minibots.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supervisor = minibots.supervisor:main',
        ],
    },
)