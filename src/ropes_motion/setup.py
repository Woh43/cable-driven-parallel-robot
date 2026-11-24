from setuptools import find_packages, setup


package_name = "ropes_motion"

setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/trajectory.yaml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Project owner",
    maintainer_email="maintainer@example.com",
    description="Cartesian waypoint planning and trajectory execution for the three-cable robot.",
    license="Proprietary",
    entry_points={"console_scripts": ["trajectory_server = ropes_motion.trajectory_server:main"]},
)
