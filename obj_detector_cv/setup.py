from setuptools import find_packages, setup

package_name = 'obj_detector_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Boccaccio',
    maintainer_email='aboccacc@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obj_detector_cv = obj_detector_cv.obj_detector_cv:main',
            'cv_subscriber = obj_detector_cv.cv_subscriber_node:main',
            'cv_subscriber = obj_detector_cv.cv_backend:main',
            'cv_subscriber = obj_detector_cv.cv_subscriber_node:main',
        ],
    },
)
