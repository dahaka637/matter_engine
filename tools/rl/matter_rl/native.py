"""Importa a ponte C++ (matter_rl_native) da build do RL."""
from __future__ import annotations

import sys

from . import paths


def load():
    directory = str(paths.NATIVE_DIR)
    if directory not in sys.path:
        sys.path.insert(0, directory)
    try:
        import matter_rl_native  # type: ignore
    except ImportError as error:  # pragma: no cover - mensagem para o usuario
        raise SystemExit(
            f"modulo nativo ausente em {paths.NATIVE_DIR} ({error}); rode tools/rl/setup.sh") from error
    return matter_rl_native
