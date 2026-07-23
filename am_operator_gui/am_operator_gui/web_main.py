"""Console entry point for the local web operator UI."""
import os
import webbrowser
from threading import Timer

import uvicorn


def main() -> int:
    port = int(os.environ.get('AM_OPERATOR_WEB_PORT', '8000'))
    url = f'http://127.0.0.1:{port}'
    if os.environ.get('AM_OPERATOR_WEB_NO_BROWSER', '').strip().lower() not in {'1', 'true', 'yes'}:
        Timer(0.8, lambda: webbrowser.open(url)).start()
    uvicorn.run('am_operator_gui.web_app:app', host='127.0.0.1', port=port, reload=False)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
