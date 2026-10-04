"""DroneKit 2.9.2 still uses ``collections.MutableMapping`` (removed in Python 3.10).

Import this module before ``dronekit``; it is the same patch that the course
scripts in 01-05 carry at the top of every file.
"""

import collections
import collections.abc

if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]
