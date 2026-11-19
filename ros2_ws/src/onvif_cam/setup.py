import subprocess
from setuptools import Extension, setup

package_name = 'onvif_cam'


def _build_fast_rtsp_ext():
    try:
        subprocess.run(
            ['pkg-config', '--exists', 'libavformat', 'libavcodec', 'libavutil', 'libswscale'],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:  # noqa: BLE001
        return []
    return [
        Extension(
            f'{package_name}._fast_rtsp',
            sources=[f'{package_name}/_fast_rtsp.c'],
            libraries=['avformat', 'avcodec', 'avutil', 'swscale'],
            extra_compile_args=['-O2'],
        )
    ]


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    ext_modules=_build_fast_rtsp_ext(),
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
