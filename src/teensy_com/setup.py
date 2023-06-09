from setuptools import setup

package_name = 'teensy_com'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', "pyserial"],
    zip_safe=True,
    maintainer='kriss',
    maintainer_email='kriss-aleksandrs.vasermans@rtu.lv',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teensy = teensy_com.teensy_publisher:main',
            'gps_publisher = teensy_com.gps_publisher:main',
            'simulator = teensy_com.simulator:main'
        ],
    },
)
