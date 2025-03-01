from setuptools import setup

package_name = 'kf_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpatnaik',
    maintainer_email='karishma5528@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['talker = kf_tutorial.subscribe_px4_velocity:main',
        'estimator = kf_tutorial.kf_example_1d:main'
        ],
    },
)
