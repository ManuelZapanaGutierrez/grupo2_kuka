from setuptools import find_packages, setup

package_name = 'grupo02_robot_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Grupo 02',
    maintainer_email='manuel@todo.todo',
    description='Cinematica directa e inversa KUKA KR 7 R900-3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fk_node = grupo02_robot_kinematics.fk_node:main',
            'ik_node = grupo02_robot_kinematics.ik_node:main',
        ],
    },
)
