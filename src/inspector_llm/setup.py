from setuptools import find_packages, setup
from glob import glob

package_name = 'inspector_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/schemas', glob('inspector_llm/schemas/*.json')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='shubham',
    maintainer_email='shubhambarge.dev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_executor = inspector_llm.mission_executor:main',
            'llm_bridge = inspector_llm.llm_bridge:main',
        ],
    },
)
