from setuptools import find_packages, setup

package_name = 'my_ur10e_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='UR10e control package',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_ros2 = my_ur10e_control.ur_ros2:main',
        ],
    },
)


# package_name = 'my_ur10e_control'

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
#     maintainer='zzz',
#     maintainer_email='zzz@todo.todo',
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
