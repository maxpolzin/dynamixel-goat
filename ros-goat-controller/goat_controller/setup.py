from setuptools import setup

package_name = 'goat_controller'

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
    maintainer='rosblox',
    maintainer_email='info@rosblox.com',
    description='GOAT controller package.',
    license='BSD3',
    entry_points={
        'console_scripts': [
                'goat_controller = goat_controller.goat_controller:main',
        ],
    },
)
