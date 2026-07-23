"""Shared, UI-independent persistence for the operator applications."""

import json
from pathlib import Path
from typing import Any


class ConfigStore:
    """Read and write the operator configuration without depending on a UI toolkit."""

    def __init__(self, path: Path, legacy_path: Path | None = None) -> None:
        self.path = path
        self.legacy_path = legacy_path

    def load(self) -> dict[str, Any]:
        for candidate in (self.path, self.legacy_path):
            if candidate is None:
                continue
            try:
                with candidate.open(encoding='utf-8') as stream:
                    data = json.load(stream)
            except FileNotFoundError:
                continue
            except (OSError, json.JSONDecodeError):
                return {}
            return data if isinstance(data, dict) else {}
        return {}

    def save(self, data: dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open('w', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2, sort_keys=True)
            stream.write('\n')
