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
        'left_middle_publisher = leg_hexapod.controller_leg_left_mid:main',
        'left_up_publisher = leg_hexapod.controller_leg_left_up:main',
        'left_down_publisher = leg_hexapod.controller_leg_left_down:main',
        'right_up_publisher = leg_hexapod.controller_leg_right_up:main',
        'right_middle_publisher = leg_hexapod.controller_leg_right_mid:main',
        'right_down_publisher = leg_hexapod.controller_leg_right_down:main',
        'stand_right_down_publisher = leg_hexapod.stand_in_loop_rd:main',
        'stand_left_down_publisher = leg_hexapod.stand_in_loop_ld:main',
        'stand_right_up_publisher = leg_hexapod.stand_in_loop_ru:main',
        'stand_left_up_publisher = leg_hexapod.stand_in_loop_lu:main',
        'stand_right_mid_publisher = leg_hexapod.stand_in_loop_rm:main',
        'stand_left_mid_publisher = leg_hexapod.stand_in_loop_lm:main',
        'stand = leg_hexapod.stand:main',
        'main_controller = leg_hexapod.main_controller:main',
        'control_v2_main = leg_hexapod.main_controller_v2:main',
        'servo_theta_read = leg_hexapod.servo_subscriber:main'
        ],
    },
)
