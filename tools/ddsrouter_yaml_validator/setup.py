"""Setup file to install ddsrouter_yaml_validator module."""
from setuptools import setup


package_name = 'ddsrouter_yaml_validator'

setup(
    name=package_name,
    version='3.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [package_name + '/ddsrouter_config_schema.json']),
        ('share/' + package_name, [package_name + '/full_example.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eprosima',
    maintainer_email='juanlopez@eprosima.com',
    description='Tool used for validating DDS-Router configuration files',
    license='Apache License, Version 2.0',
    extras_require={
        'test': [
            'pytest',  # Add your testing dependencies here
        ],
    },
    entry_points={
        'console_scripts': [
            'ddsrouter_yaml_validator = ddsrouter_yaml_validator.ddsrouter_yaml_validator:main',
        ],
    },
)
