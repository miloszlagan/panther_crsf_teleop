from setuptools import find_packages, setup

package_name = 'panther_crsf_teleop'

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
    maintainer='mily',
    maintainer_email='milosz.lagan@husarion.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panther_crsf_teleop = panther_crsf_teleop.panther_crsf_teleop:main'
        ],
    },
)
