from setuptools import setup

package_name = 'pilsbot_log_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='info@pilsbot.de',
    description='Tools for Pilsbot logs',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convert_legacy_bag = pilsbot_log_tools.convert_legacy_bag:main',
        ],
    },
)
