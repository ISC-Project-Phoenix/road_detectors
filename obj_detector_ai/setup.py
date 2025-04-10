from setuptools import find_packages, setup

package_name = 'obj_detector_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','opencv-python','cv_bridge','torch','ultralytics'],
    zip_safe=True,
    maintainer='andy',
    maintainer_email='Andrew@Ealovega.dev',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obj_detector_ai = obj_detector_ai.obj_detector_ai:main',
            'image_publisher_node = obj_detector_ai.image_publisher_node:main',
            'image_publisher_nodeTest = obj_detector_ai.image_publisher_nodeTest:main',
            'yolo_subscriber_node = obj_detector_ai.yolo_subscriber_node:main',
            'yolo_subscriber_node_new = obj_detector_ai.yolo_subscriber_node_new:main',
            'yolo_publisher_node = obj_detector_ai.yolo_publisher_node:main',
            'image_save_node = obj_detector_ai.save_img:main',

        ],
    },
)
