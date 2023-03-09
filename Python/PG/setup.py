from setuptools import setup, find_packages

setup(
    name='PG',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        'numpy==1.22.3',
        'opencv-python==4.6.0.66'
    ],
    author='Ertugrul Tiyek',
    author_email='ertugrultyk@gmail.com',
    description='Papoulis Gerchberg Algorithm to upsample 3D lidar data',
)