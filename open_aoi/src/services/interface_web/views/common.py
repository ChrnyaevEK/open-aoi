from nicegui import ui, app

from src.enums import AccessorEnum
from src.models import Accessor
from src.exceptions import AuthException
from src.services.interface_web.views.utils import db_get_accessor


colors = dict(primary="#3A6B35", secondary="#CBD18F")

HOME_PAGE = "/"
ACCESS_PAGE = "/access"
INSPECTION_PROFILE_PAGE = "/inspection/profile"
INSPECTION_LIVE_PAGE = "/inspection/live"
DEVICES_PAGE = "/devices"
SETTINGS_PAGE = "/settings"


def inject_commons():
    ui.colors(**colors)


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
    links_props = "flat square size=md"
    links_classes = "text-white"

    # TODO: adaptive
    with ui.header(fixed=True).classes("py-1"):
        ui.markdown("**AOI Portal** | Powered by ROS")
        ui.button("Overview", on_click=lambda: ui.open(HOME_PAGE)).classes(
            links_classes
        ).props(links_props)
        ui.button("Devices", on_click=lambda: ui.open(DEVICES_PAGE)).classes(
            links_classes
        ).props(links_props)
        ui.button("Settings", on_click=lambda: ui.open(SETTINGS_PAGE)).classes(
            links_classes
        ).props(links_props)
        ui.button(
            "Inspection profiles", on_click=lambda: ui.open(INSPECTION_PROFILE_PAGE)
        ).classes(links_classes).props(links_props)
        with ui.button(
            "Inspection", on_click=lambda: ui.open(INSPECTION_LIVE_PAGE)
        ).classes(links_classes).props(links_props):
            # ui.badge("offline", color="grey").classes('ml-1').props("rounded")
            ui.badge("online", color="red").classes("ml-1").props("rounded")
        ui.space()
        ui.button("Logout", on_click=_handle_logout_request).classes(
            links_classes
        ).props(links_props)


def inject_container():
    ui.left_drawer()
    ui.right_drawer()


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
