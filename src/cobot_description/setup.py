from setuptools import setup

package_name = 'cobot_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aryan',
    maintainer_email='aryan@bennettuniversity.edu.in',
    description='6-DOF Teleoperated Cobot — URDF, RViz, Launch, Demo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'demo_motion          = cobot_description.demo_motion:main',
            'impedance_controller = cobot_description.impedance_controller:main',
            'teleop_node          = cobot_description.teleop_node:main',
            'vision_node          = cobot_description.vision_node:main',
        ],
    },
)
