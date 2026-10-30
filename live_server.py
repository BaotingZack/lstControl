"""Compatibility shim — implementation in ``web.server``."""

from web import server as _server
from web.server import *  # noqa: F403

for _name in dir(_server):
    if _name.startswith('_') and not _name.startswith('__'):
        globals()[_name] = getattr(_server, _name)

del _server, _name
