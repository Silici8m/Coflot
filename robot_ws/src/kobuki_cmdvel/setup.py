from setuptools import find_packages, setup

package_name = 'kobuki_cmdvel'

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
    maintainer='ocha',
    maintainer_email='goomog@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'kobuki_cmdvel = kobuki_cmdvel.kobuki_cmdvel_function:main'
        ],
    },
)
