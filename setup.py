from setuptools import setup

package_name = 'arm3dof'

setup(
    name=package_name,
    version='0.0.1',

    # You have a single module file: src/move_demo.py
    packages=[],
    py_modules=['move_demo'],
    package_dir={'': 'src'},

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deniz',
    maintainer_email='none@none.com',
    description='3-DOF robot arm demo controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'move_demo = move_demo:main',
        ],
    },
)
