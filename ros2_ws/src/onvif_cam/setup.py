from setuptools import setup

package_name = 'onvif_cam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='ONVIF/RTSP IP camera driver with zoom control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = onvif_cam.camera_node:main',
            'cam_gui = onvif_cam.gui_node:main',
        ],
    },
)
