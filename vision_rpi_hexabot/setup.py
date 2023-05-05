from setuptools import setup

package_name = 'vision_rpi_hexabot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michal',
    maintainer_email='michal@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

                'publisher_rpi_node = vision_rpi_hexabot.publisher:main',
                'listener_rpi_node = vision_rpi_hexabot.subsriber:main',

                           ],
                },

)
