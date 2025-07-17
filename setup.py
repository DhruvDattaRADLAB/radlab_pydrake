from setuptools import setup, find_packages

setup(
    name='radlab_pydrake',
    version='0.2.9',
    packages=find_packages(),
    include_package_data=True,  # Important: allows MANIFEST.in or setup.cfg to work
    package_data={
        "radlab_pydrake.roboball_plant.roboball_urdf": [
            "package.xml",
            "*.urdf",
            "urdf/*.urdf",   # for lumpy + shell
            "meshes/*.obj",
        ],
    },
    install_requires=[],
    author='Your Name',
    author_email='your@email.com',
    description='A simple custom package',
    url='https://github.com/yourusername/my_package',  # Optional
)
