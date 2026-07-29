from setuptools import setup

package_name = 'mavros_extras'

setup(
    name=package_name,
    version='2.14.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Vladimir Ermakov',
    author_email='vooon341@gmail.com',
    maintainer='Vladimir Ermakov',
    maintainer_email='vooon341@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
    ],
    description='Extra nodes and plugins for MAVROS',
    license='Triple licensed under GPLv3, LGPLv3 and BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terrain_tile_server = mavros_extras.terrain_server_node:main',
        ],
    },
)
