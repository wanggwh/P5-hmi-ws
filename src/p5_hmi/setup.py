from setuptools import find_packages, setup

package_name = 'p5_hmi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/p5_hmi']),
        ('share/p5_hmi', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'hmi = p5_hmi.hmi:main',
        ]
    },
)
