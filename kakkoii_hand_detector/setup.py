from setuptools import find_packages, setup

package_name = 'kakkoii_hand_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TN-kakkoii',
    description='MediaPipe Hands による手の検出ノード',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hand_detector = kakkoii_hand_detector.hand_detector_node:main',
        ],
    },
)
