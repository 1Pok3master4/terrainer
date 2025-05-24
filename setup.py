from setuptools import find_packages, setup

package_name = 'terrain_mapper'

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
    maintainer='mocha1410',
    maintainer_email='amogha_ss@ph.iitr.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "terrain_node = terrain_mapper.node:main",
            "control_node = terrain_mapper.controller:main",
            "test = terrain_mapper.nodde_for_2d:main",
            "terrainer = terrain_mapper.test_controller:main",
            "feed = terrain_mapper.live_feed:main"
        ],
    },
)
