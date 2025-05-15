from setuptools import find_packages, setup

package_name = 'tf_server'

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
    maintainer='zulal',
    maintainer_email='zulaluludogan01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_server = tf_server.tf_server:main',
            'tf_server2 = tf_server.tf_server2:main',  # <- Add this line
        ],
    },
)
