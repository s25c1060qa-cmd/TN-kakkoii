from setuptools import find_packages, setup

package_name = 'kakkoii_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takeguchi-kota',
    maintainer_email='takeguchi@todo.todo',
    description='Vision and arm control nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'color_detector = kakkoii_vision.color_detector_publisher:main',
            'block_detector = kakkoii_vision.block_shape_detector:main',
            'mediapipe_tracker = kakkoii_vision.test_mediapipe_publisher:main',
            'arm_controller = kakkoii_vision.mediapipe_arm_controller:main',
            'block_arm_controller = kakkoii_vision.mediapipe_brock_arm_controller:main',
        ],
    },
)
