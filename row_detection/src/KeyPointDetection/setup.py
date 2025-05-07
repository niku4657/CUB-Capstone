#!/usr/bin/env python3

import os
from setuptools import setup, find_packages

# Get the long description from the README file
with open(os.path.join(os.path.abspath(os.path.dirname(__file__)), "README")) as f:
    longDescription = f.read()


# We do not include tensorflow as a install_requires because we could use
# either tensorflow and tensorflow-gpu and there's no good way to say we only
# require one or the other
setup(
    name="predict/exPointDetection",
    version="0.1",
    description="Exodus Point Detection",
    long_description=longDescription,
    url="https://bitbucket.trimble.tools/projects/TML/repos/exodus/browse",
    author="Robert Banfield",
    author_email="robert_banfield@trimble.com",
    packages=["predict/exPointDetection"],
    install_requires=["numpy", "lxml"],
    scripts=[
        "predict/__main__.py"
    ],
)
