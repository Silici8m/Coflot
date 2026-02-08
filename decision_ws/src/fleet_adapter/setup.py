from setuptools import find_packages, setup

package_name = 'fleet_adapter'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexis',
    maintainer_email='alex5901@outlook.fr',
    description='Interface d\'adaptation pour la gestion de flotte.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fleet_adapter = fleet_adapter.fleet_adapter:main',
        ],
    },
)