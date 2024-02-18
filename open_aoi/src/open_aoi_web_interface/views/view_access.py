import asyncio
import logging
from typing import Optional
from nicegui import ui, app
from fastapi.responses import RedirectResponse

from open_aoi.enums import AccessorEnum
from open_aoi.exceptions import AuthException
from open_aoi_web_interface.views.utils import db_get_accessor
from open_aoi_web_interface.views.common import HOME_PAGE

logger = logging.getLogger("ui.access")


def _handle_access_request(password_input: ui.input):
    operator = db_get_accessor(AccessorEnum.OPERATOR)
    if operator is None:
        return

    administrator = db_get_accessor(AccessorEnum.ADMINISTRATOR)
    if administrator is None:
        return

    try:
        operator.test_credentials(password=password_input.value)
    except AuthException as e:
        logger.info("Failed to test credentials against operator accessor")
        try:
            administrator.test_credentials(password=password_input.value)
        except AuthException as e:
            logger.info("Failed to test credentials against administrator accessor")
            ui.notify("Invalid credentials", type="negative")
            return
        else:
            # Allow admin access
            administrator.grant_access(app.storage.user)
            ui.open(HOME_PAGE)
            return
    else:
        # Allow operator access
        operator.grant_access(app.storage.user)
        ui.open(HOME_PAGE)
        return


async def view() -> Optional[RedirectResponse]:
    operator = db_get_accessor(AccessorEnum.OPERATOR)
    if operator is None:
        return

    administrator = db_get_accessor(AccessorEnum.ADMINISTRATOR)
    if administrator is None:
        return

    # Do not allow logged in users to access page
    try:
        operator.assert_access(app.storage.user)
    except AuthException:
        pass
    else:
        return RedirectResponse(HOME_PAGE)

    try:
        administrator.assert_access(app.storage.user)
    except AuthException:
        pass
    else:
        return RedirectResponse(HOME_PAGE)

    # Render page
    with ui.card().classes("absolute-center w-80"):
        with ui.row().classes("w-full justify-between items-center"):
            ui.markdown("**Enter credentials**")
            info = ui.button(icon="question_mark").props("flat round size=xs")
            info.tooltip("To access system please enter your credentials")
        password_input = ui.input(placeholder="Password", password=True).classes(
            "w-full"
        )
        ui.button(
            "Continue",
            color="primary",
            on_click=lambda: _handle_access_request(password_input),
        ).classes("w-full")

    with ui.header(fixed=True).classes("py-1"):
        ui.markdown("**AOI Portal** | Powered by ROS")
    with ui.footer(fixed=True).classes("py-2"):
        ui.label("Created by @Cherniaev.public")
