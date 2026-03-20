# Python Style Guide — Volume Cartographer

## Scope

Applies to:
- Python bindings (`python/` module, pybind11)
- Python utility scripts
- Test scripts

## Formatting

- **Black** for formatting (line length 88)
- **isort** for import ordering
- **ruff** or **flake8** for linting

Run before committing:
```shell
black .
isort .
```

## Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| Module | `snake_case` | `volcart`, `volume_pkg` |
| Class | `PascalCase` | `VolumePkg`, `Segmentation` |
| Function/Method | `snake_case` | `get_volume()`, `load_slice()` |
| Variable | `snake_case` | `slice_index`, `uv_map` |
| Constant | `UPPER_SNAKE_CASE` | `MAX_CACHE_SIZE` |
| Private | leading underscore | `_internal_state` |

## Bindings (pybind11)

- Python-facing names should follow Python `snake_case` conventions even when wrapping `camelCase` C++ methods
- Expose docstrings for all bound classes and methods
- Use `py::arg("name")` for all parameters to support keyword argument calls
- Return numpy arrays (not raw buffers) for image/volume data where possible

## Type Hints

- All public functions and methods must have type annotations
- Use `from __future__ import annotations` for forward references
- Use `Optional[X]` / `X | None` (Python 3.10+ style preferred)

```python
def load_volume(path: str, cache_size: int = 256) -> Volume:
    ...
```

## Imports

Order (enforced by isort):
1. Standard library
2. Third-party (`numpy`, `opencv-python`, etc.)
3. Local / volcart bindings

No wildcard imports (`from module import *`).

## Error Handling

- Raise specific exceptions (`ValueError`, `FileNotFoundError`, `RuntimeError`)
- Do not catch and suppress exceptions silently
- C++ exceptions from pybind11 bindings should propagate naturally unless you need to translate them

## Testing

- Tests use `pytest`
- Test files named `test_{feature}.py`
- Mirror the C++ test coverage for any bound functionality
