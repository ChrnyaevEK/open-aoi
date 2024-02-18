import logging
from typing import Optional

from nicegui import ui
from fastapi.responses import RedirectResponse

from open_aoi.exceptions import AuthException
from open_aoi_web_interface.views.common import (
    inject_header,
    ACCESS_PAGE,
    ensure_access_guard,
)

logger = logging.getLogger("ui.home")


def view() -> Optional[RedirectResponse]:
    try:
        ensure_access_guard()
    except AuthException:
        return RedirectResponse(ACCESS_PAGE)

    inject_header()
