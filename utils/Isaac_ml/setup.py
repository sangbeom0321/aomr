from setuptools import setup

package_name = 'isaac_ml'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    author_email='you@youremail.com',
    maintainer='YourFirstname Lastname',
    maintainer_email='your@youremail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A simple ROS2 Python package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'four_ws_control = scripts.four_ws_control:main',
            'four_ws_control_pos = scripts.four_ws_control_pos:main',
            'keyboard_teleop = scripts.keyboard_teleop:main',
            'state_machine = scripts.state_machine:main',
            'drone_local_path = scripts.drone_local_path:main',
            'balance_joint_lock = scripts.balance_joint_lock:main',
        ],
    },
)