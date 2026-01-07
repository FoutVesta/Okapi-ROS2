# SPDX-License-Identifier: Apache-2.0
# __init__.py for ROS 2 Python package
# This file initializes the module namespace for rfidbot_tags_reader

from importlib import metadata
from . import llrp
from . import llrp_decoder
from . import llrp_errors
from . import llrp_proto
from . import util
# Explicitly expose public submodules
__all__ = [
    "llrp",
    "llrp_decoder",
    "llrp_errors",
    "llrp_proto",
    "util",
    "inventory"
]

# Version management (uses package metadata when installed with ament)
try:
    __version__ = metadata.version("rfidbot_tags_reader")
except metadata.PackageNotFoundError:
    __version__ = "0.0.1"

