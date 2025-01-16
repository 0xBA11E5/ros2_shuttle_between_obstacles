from setuptools import find_packages, setup

package_name = 'shuttle_between_obstacles'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lubun',
    maintainer_email='l4310727@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_line = shuttle_between_obstacles.follow_line:main',
            'follow_object = shuttle_between_obstacles.follow_object:main',
            'shuttle_between_obstacles = shuttle_between_obstacles.shuttle_between_obstacles:main'
        ],
    },
)
