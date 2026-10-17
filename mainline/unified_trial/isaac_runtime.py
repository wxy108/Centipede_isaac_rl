"""Legacy model/command dependencies, imported only after AppLauncher starts.

Paths and Warp initialization belong to cli.py. Importing the public package,
its input validator or world-pitch controller does not import this module.
"""

import common
from mechanics import scene_diagnostics
from optimization import command_cache_nonnegative as command_cache_module


CommandCache = command_cache_module.CommandCache

__all__ = ["common", "scene_diagnostics", "command_cache_module", "CommandCache"]
