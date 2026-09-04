from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


PACKAGE_NAME = "vgm_runtime"
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (
            f"share/{PACKAGE_NAME}/config",
            [
                str(REPOSITORY_ROOT / "config" / "robot_skill.schema.json"),
                str(REPOSITORY_ROOT / "config" / "safety_policy.json"),
                str(REPOSITORY_ROOT / "config" / "phase_1_scene.json"),
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="malovicfilip",
    maintainer_email="malovicfilip1@gmail.com",
    description="Safety-bounded runtime for voice-grounded manipulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "intent_cli = vgm_runtime.cli:main",
            "execution_gate = vgm_runtime.gate_cli:main",
            "skill_gateway = vgm_runtime.ros_gateway:main",
            "voice_cli = vgm_runtime.voice_cli:main",
        ]
    },
)
