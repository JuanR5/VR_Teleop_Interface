from setuptools import find_packages, setup

package_name = 'middle_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/zed_vr_conexion.launch.py']),
    ],
    package_data={
        # Install all launch files in the 'launch' directory
        package_name + '/launch': ['*.launch.py'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juan',
    maintainer_email='juan_p.rivera@uao.edu.co',
    description='MIddle nodes used for testing, data conversion or republishing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zedStream = middle_nodes.zed_stream:main',
            'zedimage_bridge = middle_nodes.zedimage_bridge:main',

        ],
    },
)
