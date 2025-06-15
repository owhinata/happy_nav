from setuptools import find_packages, setup

package_name = 'happy_nav'

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
    maintainer='KatsumiOuwa',
    maintainer_email='ouwa@emtechs.co.jp',
    description='happy navigate',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'happy_teleop_node = happy_nav.happy_teleop_node:main',
            'happy_move_node = happy_nav.happy_move_node:main',
        ],
    },
)
