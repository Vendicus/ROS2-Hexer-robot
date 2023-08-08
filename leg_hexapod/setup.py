from setuptools import setup

package_name = 'leg_hexapod'
submodules = "leg_hexapod/submod"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michal',
    maintainer_email='mich.aksamit@wp.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'test_node = leg_hexapod.test:main',
        'servo_theta_read = leg_hexapod.servo_subscriber:main',
        'v2_servo_theta_read = leg_hexapod.servo_sub_v2:main'
        ],
    },
)
