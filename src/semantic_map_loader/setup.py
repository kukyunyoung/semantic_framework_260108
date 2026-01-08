from setuptools import find_packages, setup

package_name = 'semantic_map_loader'

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
    maintainer='n',
    maintainer_email='n@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'semantic_map_loader = semantic_map_loader.semantic_map_loader:main',
            'fake_robot = semantic_map_loader.fake_robot_node:main',
            'semantic_nav = semantic_map_loader.semantic_nav:main',
        ],
    },
)
