from setuptools import setup

package_name = 'projekt2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/projekt2']),
    ('share/projekt2', ['package.xml']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TwojeImie',
    maintainer_email='twojemail@example.com',
    description='Opis pakietu',
    license='Licencja',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'projekt2 = projekt2.projekt2:main',
        ],
    },
)