import os
from glob import glob
from setuptools import setup

package_name = 'test_bench'

setup(
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ]
)
