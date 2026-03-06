# dexim-core

This repository consolidates legacy `core-*`, `shared_messages`, and `ts-spatial`
sub-projects into one import namespace: `dexim.core`.

## Environment Installation

This repository uses a root Pixi workspace with Python 3.10 as the baseline.
`pinocchio` is installed from `conda-forge` in the same environment.
For local development, the root `pixi.toml` solves only the host platform
(currently `win-64`).

1. Install **pixi** (if not already installed):

```powershell
# Windows (PowerShell)
iwr -useb https://pixi.sh/install.ps1 | iex
```

```bash
# Linux / macOS
curl -fsSL https://pixi.sh/install.sh | sh
```

> See https://pixi.sh/latest/#installation for other options.

2. Resolve the root environment:

```bash
pixi install
```

3. Validate imports:

```bash
pixi run check-import
```

4. Optional: install PyTorch (CUDA 13.0). This is not managed by this project:

```bash
pixi run pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130
```

> For other CUDA versions or CPU-only builds, see https://pytorch.org/get-started/locally/

## Module Layout

- `dexim.core.config`
- `dexim.core.model`
- `dexim.core.nodes`
- `dexim.core.robot_interface`
- `dexim.core.utility`
- `dexim.core.messages`
- `dexim.core.spatial`

## Why This Layout

The legacy folders under `src/dexim/core` were standalone packages (each with its
own `pyproject.toml`). The extracted layout keeps those capabilities but exposes
them as modules in a single package so you can install and import from one place.

## Example Imports

```python
from dexim.core.config import ControlNodeConfig
from dexim.core.nodes import ManagedNode
from dexim.core.messages import TopicBuilder
from dexim.core.spatial import Transform3D
```