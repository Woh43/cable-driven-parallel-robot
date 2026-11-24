from setuptools import setup


package_name = "ropes_kinematics"

setup(
    name=package_name,
    version="0.5.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project owner",
    maintainer_email="maintainer@example.com",
    description="Geometric kinematics and cable feasibility for the three-cable robot.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "kinematics_server = ropes_kinematics.kinematics_server:main",
        ],
    },
)
