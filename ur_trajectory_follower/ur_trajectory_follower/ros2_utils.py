from typing import Iterable, List


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'y', 'on'}


def as_float_list(value, fallback: Iterable[float]) -> List[float]:
    if isinstance(value, str):
        raw_items = value.strip().strip('[]').split(',')
        try:
            parsed = [float(item.strip()) for item in raw_items if item.strip()]
        except ValueError:
            return list(fallback)
        return parsed if parsed else list(fallback)
    try:
        return [float(item) for item in value]
    except TypeError:
        return list(fallback)


def as_string_list(value) -> List[str]:
    if isinstance(value, str):
        stripped = value.strip()
        if stripped.startswith('[') and stripped.endswith(']'):
            stripped = stripped[1:-1]
        return [item.strip().strip("'\"") for item in stripped.split(',') if item.strip()]
    try:
        return [str(item) for item in value]
    except TypeError:
        return []
