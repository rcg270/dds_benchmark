from setuptools import find_packages, setup

package_name = 'dds_benchmark'

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
    maintainer='robby',
    maintainer_email='robbychingjong@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'talker_metrics = dds_benchmark.talker_metrics:main',
            # 'listener_metrics = dds_benchmark.listener_metrics:main',
            'dds_talker = dds_benchmark.dds_talker:main',
            'dds_listener = dds_benchmark.dds_listener:main',
        ],
    },
)
