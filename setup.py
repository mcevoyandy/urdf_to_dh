from setuptools import setup
import os
from glob import glob

package_name = 'urdf_to_dh'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name), glob('launch/*.launch.py')),
        (os.path.join('share/', package_name), glob('launch/*.rviz')),
        (os.path.join('share/', package_name, 'urdf'), ['urdf/random.urdf'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andy',
    maintainer_email='andy.mcevoy@sslmda.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_dh = urdf_to_dh.generate_dh:main'
        ],
    },
)
