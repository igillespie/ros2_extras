from setuptools import setup

package_name = 'mast_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'adafruit-circuitpython-servokit'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='ian_gillespie@mac.com',
    description='Control the mast of the rover, pan and tilt allowed.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mast_control = mast_control.mast_control:main',
        ],
    },
)