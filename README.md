# dexim-core

This repository consolidates legacy `core-*`, `shared_messages`, and `ts-spatial`
sub-projects into one import namespace: `dexim_core`.

## Environment Installation

1. Create a conda environment:

```bash
conda create -n dexim python==3.10
conda activate dexim
```

2. Install `uv` in the conda environment:

```bash
conda install conda-forge::uv
```

3. Install `pinocchio` from conda-forge:

```bash
conda install conda-forge::pinocchio
```

4. Install PyTorch (CUDA 13.0) — **not managed by `pyproject.toml`**, install manually:

```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu130
```

> For other CUDA versions or CPU-only builds, see https://pytorch.org/get-started/locally/

5. Install all project dependencies with `uv pip`:

```bash
uv pip install -e .
```

## Module Layout

- `dexim_core.config`
- `dexim_core.model`
- `dexim_core.nodes`
- `dexim_core.robot_interface`
- `dexim_core.utility`
- `dexim_core.messages`
- `dexim_core.spatial`

## Why This Layout

The legacy folders under `src/dexim_core` were standalone packages (each with its
own `pyproject.toml`). The extracted layout keeps those capabilities but exposes
them as modules in a single package so you can install and import from one place.

## Example Imports

```python
from dexim_core.config import ControlNodeConfig
from dexim_core.nodes import ManagedNode
from dexim_core.messages import TopicBuilder
from dexim_core.spatial import Transform3D
```