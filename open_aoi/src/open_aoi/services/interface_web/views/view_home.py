import logging
from typing import Optional

from nicegui import ui
from fastapi.responses import RedirectResponse

from exceptions import AuthException
from services.interface_web.views.common import (
    inject_commons,
    inject_header,
    inject_container,
    ACCESS_PAGE,
    ensure_access_guard,
)

logger = logging.getLogger("ui.home")


def view() -> Optional[RedirectResponse]:
    try:
        ensure_access_guard()
    except AuthException:
        return RedirectResponse(ACCESS_PAGE)

    # Render page
    ui.page_title("Home | AOI Portal")
    inject_commons()
    inject_header()
    inject_container()
