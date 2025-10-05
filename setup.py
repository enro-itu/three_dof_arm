from setuptools import setup

package_name = 'arm3dof'

setup(
	name=package_name,
	version='0.0.1',
	packages=['src'],
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name],
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='deniz',
	maintrainer_email='none@none.com',
	description='EDOF robot arm demo controller',
	license='MIT',
	entry_points={
		'console_scripts': [
			'move_demo = src.move_demo:main'
		],
	},
)
