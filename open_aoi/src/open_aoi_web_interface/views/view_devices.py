import logging
from typing import Optional

from nicegui import ui
from fastapi.responses import RedirectResponse

from open_aoi.exceptions import AuthException
from open_aoi.models import TITLE_LIMIT, DESCRIPTION_LIMIT
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

    with ui.element("div").classes("w-full grid grid-cols-1 md:grid-cols-12"):
        with ui.row().classes("col-span-1 md:col-span-8"):
            ui.label("Devices")
            # Camera title
            camera_title = (
                ui.input(
                    label="Camera title",
                    placeholder=f"Enter any value... [{TITLE_LIMIT}]",
                    on_change=lambda e: camera_title_display.set_text(
                        f"[{len(camera_title.value)}/{TITLE_LIMIT}] {camera_title.value}"
                    ),
                    validation={
                        "Title is too long": lambda value: len(value) <= TITLE_LIMIT
                    },
                )
                .classes("w-full")
                .props("clearable")
            )
            camera_title_display = ui.label("").classes("text-secondary")

            camera_description = (
                ui.input(
                    label="Camera description",
                    placeholder=f"Enter any value... [{DESCRIPTION_LIMIT}]",
                    on_change=lambda e: camera_description_display.set_text(
                        f"[{len(camera_description.value)}/{DESCRIPTION_LIMIT}] {camera_description.value}"
                    ),
                    validation={
                        "Description is too long": lambda value: len(value)
                        <= DESCRIPTION_LIMIT
                    },
                )
                .classes("w-full")
                .props("clearable")
            )
            camera_description_display = ui.label("").classes("text-secondary")

        with ui.row().classes("col-span-1 md:col-span-4"):
            ui.label("Image")
