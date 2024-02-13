import logging
from typing import Optional

from nicegui import ui
from fastapi.responses import RedirectResponse

from open_aoi.exceptions import AuthException
from open_aoi.models import TITLE_LIMIT
from open_aoi_web_interface.views.common import (
    inject_header,
    ACCESS_PAGE,
    ensure_access_guard,
)

logger = logging.getLogger("ui.devices")


def view() -> Optional[RedirectResponse]:
    try:
        ensure_access_guard()
    except AuthException:
        return RedirectResponse(ACCESS_PAGE)

    inject_header()

    with ui.row().classes("w-full grid grid-cols-3 md:grid-cols-1 gap-2"):
        with ui.column().classes("col-span-2 md:col-span-1"):
            ui.markdown("#### Devices")
            camera_title = (
                ui.input(
                    label="Camera title",
                    placeholder="Enter any value...",
                    on_change=lambda e: camera_title_display.set_text(
                        f"Camera: {camera_title.value} [{len(camera_title.value)}/{TITLE_LIMIT}]"
                    ),
                    validation={
                        "Title is too long": lambda value: len(value) <= TITLE_LIMIT
                    },
                )
                .classes("w-full")
                .props("clearable")
            )
            camera_title_display = ui.label("").classes('text-secondary')

        with ui.column().classes("col-span-1"):
            ui.label("here")
