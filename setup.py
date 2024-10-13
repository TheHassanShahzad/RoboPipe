from setuptools import find_packages, setup

package_name = 'RoboPipe'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='the_hassan_shahzad',
    maintainer_email='hshahzad2005108277@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "load_arm = RoboPipe.load_arm:main",
            "arm_control = RoboPipe.arm_control:main",
            "z_sine = RoboPipe.z_sine:main",
            "random_move = RoboPipe.random_move:main",
            "camera_controller = RoboPipe.camera_controller:main"
        ],
    },
)
