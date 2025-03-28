from skbuild import setup

setup(
    name="orca",
    version="0.1.0",
    description="Python bindings for ORCA (Optimal Reciprocal Collision Avoidance) library",
    author="Your Name",
    author_email="your.email@example.com",
    packages=["orca"],
    cmake_install_dir="orca",
    cmake_args=["-DBUILD_PYTHON_BINDINGS=ON"],
    install_requires=["pybind11"],
    python_requires=">=3.7",
) 