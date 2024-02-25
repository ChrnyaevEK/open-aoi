from setuptools import setup

package_name = 'open_aoi_ros'

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
    maintainer='Cherniaev Egor',
    maintainer_email='chrnyaevek@gmail.com',
    entry_points={
        'console_scripts': [
            'image_acquisition_service = open_aoi_ros.image_acquisition_server:main'
        ],
    },
)