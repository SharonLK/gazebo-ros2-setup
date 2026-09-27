from setuptools import find_packages, setup

package_name = 'robot_logic'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
        exclude=['test'],
    ),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='sharon',
    maintainer_email='user@example.com',
    description='Robot-independent application logic',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'drive_demo = robot_logic.drive_demo:main',
            'basic_mapper = robot_logic.basic_mapper:main',
        ],
    },
)
