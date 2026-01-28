from setuptools import find_packages
from setuptools import setup

setup(
    name='myactuator_rmd',
    version='0.0.1',
    packages=find_packages(
        include=('myactuator_rmd', 'myactuator_rmd.*')),
)
