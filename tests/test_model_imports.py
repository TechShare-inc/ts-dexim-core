import os
import subprocess
import sys


def test_model_helpers_do_not_import_optimizer_dependencies() -> None:
    environment = os.environ.copy()
    environment["PYTHONPATH"] = "src"
    script = """
import sys
import dexim.core.model
from dexim.core.model import load_urdf_model

assert callable(load_urdf_model)
assert "torch" not in sys.modules
assert "dexim.core.model.optimizer" not in sys.modules
"""

    subprocess.run(
        [sys.executable, "-c", script],
        check=True,
        env=environment,
        text=True,
    )


def test_unknown_model_export_raises_attribute_error() -> None:
    script = """
import dexim.core.model as model

try:
    model.not_an_export
except AttributeError:
    pass
else:
    raise AssertionError("unknown export did not raise AttributeError")
"""

    subprocess.run([sys.executable, "-c", script], check=True, text=True)
