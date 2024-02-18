from nicegui import ui, app

from open_aoi.enums import AccessorEnum
from open_aoi.models import Accessor
from open_aoi.exceptions import AuthException
from open_aoi_web_interface.views.utils import db_get_accessor


colors = dict(primary="#3A6B35", secondary="#CBD18F")

HOME_PAGE = "/"
ACCESS_PAGE = "/access"
INSPECTION_PROFILE_PAGE = "/inspection/profile"
INSPECTION_LIVE_PAGE = "/inspection/live"
DEVICES_PAGE = "/devices"
SETTINGS_PAGE = "/settings"


def _handle_logout_request():
    def logout():
        Accessor.revoke_access(app.storage.user)
        ui.open(ACCESS_PAGE)

    with ui.dialog() as dialog, ui.card():
        ui.label("You are about to logout. Are you sure?")
        with ui.row().classes("w-full justify-end"):
            ui.button("Cancel", on_click=dialog.close, color="white")
            ui.button("Confirm", on_click=logout, color="primary")

    dialog.open()


def inject_header():
    ui.left_drawer()
    ui.right_drawer()
    with ui.header(fixed=True).classes("py-1 items-center"):
        ui.markdown("**AOI Portal** | Powered by ROS")
        ui.badge("offline", color="grey").classes('ml-1').props("rounded")
        # ui.badge("online", color="red").classes("ml-1").props("rounded")
        ui.space()
        with ui.button(icon="menu").props("flat text-color=white"):
            with ui.menu() as menu:
                ui.menu_item("Overview", lambda: ui.open(HOME_PAGE))
                ui.menu_item("Devices", lambda: ui.open(DEVICES_PAGE))
                ui.menu_item("Settings", lambda: ui.open(SETTINGS_PAGE))
                ui.menu_item("Inspection", lambda: ui.open(INSPECTION_LIVE_PAGE))
                ui.menu_item(
                    "Inspection profiles", lambda: ui.open(INSPECTION_PROFILE_PAGE)
                )
                ui.separator()
                ui.menu_item("Logout", _handle_logout_request)


def ensure_access_guard() -> None:
    operator = db_get_accessor(AccessorEnum.OPERATOR)
    if operator is None:
        raise AuthException("Access denied")

    administrator = db_get_accessor(AccessorEnum.ADMINISTRATOR)
    if administrator is None:
        raise AuthException("Access denied")

    try:
        operator.assert_access(app.storage.user)
    except AuthException:
        try:
            administrator.assert_access(app.storage.user)
        except AuthException:
            raise AuthException("Access denied")
