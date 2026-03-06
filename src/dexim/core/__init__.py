"""dexim.core — DexImitate framework foundation package.

Provides the framework-level building blocks shared across all robot packages:

    from dexim.core.spatial import Transform3D
    from dexim.core.messages import TopicBuilder
    from dexim.core.config import load_config
    from dexim.core.nodes import ManagedNode

Installation
------------
    pip install dexim-core           # core framework (no hardware deps)
    pip install dexim-core[viz]      # include viser visualizer
    # pinocchio must be installed separately via conda:
    #   conda install pinocchio -c conda-forge
"""

from __future__ import annotations

__version__ = "0.1.0"
