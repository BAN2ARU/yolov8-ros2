from setuptools import setup

package_name = 'yolov8_main'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ban2aru',
    maintainer_email='qorskfo1023@hanmail.net',
    description='yolov8 for ros2 foxy version',
    license='AGPL-3.0 license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'yolov8_node = yolov8_main.yolov8_node:main'
        ],
    },
)
