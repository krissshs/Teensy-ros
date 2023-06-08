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
            'transmitter = teensy_com.publisher:main',
            'mtr_cmd_sim = teensy_com.mtr_cmd_sim:main'
        ],
    },
)
