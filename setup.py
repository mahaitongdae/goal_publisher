from setuptools import setup

package_name = 'goal_publisher'

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
    maintainer='mht',
    maintainer_email='haitongma@g.harvard.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_publisher = goal_publisher.goal_publisher:main',
            'percept_and_publish = goal_publisher.percept_and_publish:main'
        ],
    },
)
