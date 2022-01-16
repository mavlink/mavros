from setuptools import setup

package_name = "mavros"

setup(
    name=package_name,
    version="1.6.0",
    packages=[package_name],
    # package_dir={'': 'src'},
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Vladimir Ermakov",
    author_email="vooon341@gmail.com",
    maintainer="Vladimir Ermakov",
    maintainer_email="vooon341@gmail.com",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)",
        "License :: OSI Approved :: BSD License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Helper scripts and module for MAVROS",
    license="Triple licensed under GPLv3, LGPLv3 and BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mav = mavros.cmd:cli",
        ],
    },
)
