from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'my_model_ncnn_model'), 
         [f for f in glob('my_model_ncnn_model/*') if '__pycache__' not in f])
    ],
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=True,
    maintainer='price',
    maintainer_email='malubaglorraine@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detection = model.yolo_detection:main',
        ],
    },
)
