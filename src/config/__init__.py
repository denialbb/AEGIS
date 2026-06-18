# type: ignore
"""
AEGIS Configuration Package.

Loads all ``.conf`` files from this directory and exports their names as
module-level attributes so the existing ``import config; config.X`` pattern
continues to work unchanged.

Each ``.conf`` file is plain Python — it can define constants, import numpy,
etc.  The loader runs every ``.conf`` file in sorted order and populates the
module namespace with all names defined in them.
"""

import os
import sys

_config_dir = os.path.dirname(__file__)
_module_dict = sys.modules[__name__].__dict__

for _fname in sorted(f for f in os.listdir(_config_dir) if f.endswith(".conf")):
    _fpath = os.path.join(_config_dir, _fname)
    with open(_fpath) as _f:
        exec(compile(_f.read(), _fpath, "exec"), _module_dict)
