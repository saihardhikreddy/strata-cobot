from setuptools import setup

package_name = 'strata_cobot'

# Each node installs under its file name: `ros2 run strata_cobot ore_perception.py`.
SCRIPTS = [
    'scripts/ore_perception.py',
    'scripts/arm_manipulation.py',
    'scripts/path_planning.py',
    'scripts/path_following.py',
    'scripts/package_handoff.py',
]

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='C Saihardhik Reddy',
    maintainer_email='saihardhikreddy@gmail.com',
    description='Strata Cobot course project: ore sorting with a UR7e arm and path planning for a mobile robot.',
    license='Apache-2.0',
    scripts=SCRIPTS,
)
