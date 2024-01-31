import logging
from typing import Optional

from nicegui import ui
from fastapi.responses import RedirectResponse

from ros_docker_ws.core.exceptions import AuthException
from ros_docker_ws.web_app.views.common import (
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
