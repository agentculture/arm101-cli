"""arm101.explore.default_map — bundled default self-collision map + override loader.

Ships a conservative, **permissive** baseline self-collision :class:`ReachMap`
as package data (``arm101/explore/data/default_selfcollision_map.json``): full
per-joint reachable ``(0, 4095)`` ranges and an empty ``blocked`` set, i.e.
"no self-collisions known yet". There is no hardware-measured self-collision
data at this point in the build — that is the human-gated live-hardware run
(task t11, ``arm101 arm explore``) — so this module exists purely to give
every caller of ``arm101.explore`` a map to load *before* that run happens,
and to let a later run's output override it.

:func:`load_map` is the single entry point:

* ``load_map(path)`` — load a **user-supplied** map file (e.g. the output of
  a real exploration run) via :func:`arm101.explore.reachmap.load_map_file`.
* ``load_map()`` / ``load_map(None)`` — load the **bundled default** via
  :mod:`importlib.resources`, so it works from an installed wheel with no
  reliance on ``__file__``-relative path tricks or the current working
  directory.

Loading the bundled default is read-only: it never writes to, or otherwise
mutates, the packaged asset on disk.

Zero third-party imports (stdlib only: ``importlib.resources``, ``json``) —
same discipline as the rest of ``arm101.explore``.
"""

from __future__ import annotations

import importlib.resources
import json
from pathlib import Path
from typing import Optional, Union

from arm101.explore import reachmap
from arm101.explore.types import ReachMap

#: Accepted path types for :func:`load_map`'s ``path`` argument.
PathLike = Union[str, Path]

#: Package the bundled default asset lives in.
_DEFAULT_ASSET_PACKAGE = "arm101.explore"

#: Path of the bundled default asset, relative to :data:`_DEFAULT_ASSET_PACKAGE`.
_DEFAULT_ASSET_RELATIVE_PATH = "data/default_selfcollision_map.json"


def default_map_path():
    """Return the :mod:`importlib.resources` ``Traversable`` for the bundled default asset.

    Resolved via ``importlib.resources.files(...)`` (not ``__file__``-relative
    path joining), so this also works when ``arm101`` is installed as a wheel
    (including from inside a zip/egg, where the returned object may not be a
    plain filesystem path).
    """
    return importlib.resources.files(_DEFAULT_ASSET_PACKAGE).joinpath(_DEFAULT_ASSET_RELATIVE_PATH)


def load_map(path: Optional[PathLike] = None) -> ReachMap:
    """Load a self-collision :class:`ReachMap`: *path* if given, else the bundled default.

    Parameters
    ----------
    path : str | Path | None
        If given, load the user's own map file from this path (via
        :func:`arm101.explore.reachmap.load_map_file`) — typically the output
        of a prior ``arm101 arm explore`` run. If ``None`` (the default),
        load the bundled permissive baseline self-collision map shipped as
        package data.

    Returns
    -------
    ReachMap
        The loaded map. Loading the bundled default never mutates the
        packaged asset on disk — it is read-only.
    """
    if path is not None:
        return reachmap.load_map_file(path)
    data = json.loads(default_map_path().read_text(encoding="utf-8"))
    return ReachMap.from_dict(data)


__all__ = [
    "PathLike",
    "default_map_path",
    "load_map",
]
