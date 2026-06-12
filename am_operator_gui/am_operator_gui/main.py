import sys
from typing import Optional


def main(args: Optional[list[str]] = None) -> int:
    try:
        from am_operator_gui.gui import run
    except ModuleNotFoundError as exc:
        if exc.name and exc.name.startswith('PyQt5'):
            raise SystemExit(
                'PyQt5 is required to run am_operator_gui. '
                'Install the package dependency, then rebuild/source the workspace.'
            ) from exc
        raise
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
