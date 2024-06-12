from setuptools import find_packages, setup

package_name = 'm5mover'

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
    maintainer='ltrobo',
    maintainer_email='ltrobo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'm5mover = m5mover.m5mover:main',
            'joyctl = m5mover.joyctl:main',
        ],
    },
)
